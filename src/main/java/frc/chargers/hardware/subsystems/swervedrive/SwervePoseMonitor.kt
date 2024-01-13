@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.external.frc6328.MechanicalAdvantagePoseEstimator
import frc.external.frc6328.MechanicalAdvantagePoseEstimator.TimestampedVisionUpdate
import frc.external.frc6995.NomadApriltagUtil
import frc.chargers.advantagekitextensions.recordLatency
import frc.chargers.hardware.sensors.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.chargers.wpilibextensions.kinematics.ModulePositionGroup
import org.littletonrobotics.junction.Logger.*

/**
 * A Helper class used to get the pose of an [EncoderHolonomicDrivetrain],
 * with heading-supplying utilities.
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
public class SwervePoseMonitor(
    private val drivetrain: EncoderHolonomicDrivetrain,
    poseSuppliers: List<VisionPoseSupplier>,
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseMonitor {

    /* Public API */
    public constructor(
        drivetrain: EncoderHolonomicDrivetrain,
        startingPose: UnitPose2d = UnitPose2d(),
        vararg poseSuppliers: VisionPoseSupplier,
    ): this(
        drivetrain,
        poseSuppliers.toList(),
        startingPose
    )

    public val field: Field2d = Field2d().also{ SmartDashboard.putData("Field",it) }

    override val robotPose: UnitPose2d get() = poseEstimator.latestPose.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
        if (drivetrain.gyro is ZeroableHeadingProvider){
            drivetrain.gyro.zeroHeading(pose.rotation)
        }
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier){
        this.poseSuppliers.addAll(visionSystems)
    }


    /* Private Implementation */
    private val poseEstimator = MechanicalAdvantagePoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001),
    ).also{ it.resetPose(startingPose.inUnit(meters)) }

    private val poseSuppliers = poseSuppliers.toMutableList()
    private val previousDistances: Array<Distance>
    private var lastGyroHeading = Angle(0.0)
    private var wheelDeltas = ModulePositionGroup()
    private val visionUpdates: MutableList<TimestampedVisionUpdate> = mutableListOf()


    init{
        // zeroes the previous module positions; this accounts for previous wheel travel
        // detected from the relative encoders of the swerve mod motors.
        val currentPositions = drivetrain.currentModulePositions // uses getter variable; thus, value must be fetched once
        previousDistances = arrayOf(
            currentPositions.topLeftDistance,
            currentPositions.topRightDistance,
            currentPositions.bottomLeftDistance,
            currentPositions.bottomRightDistance
        )
    }

    override fun periodic(){
        recordLatency("SwervePoseMonitorLoopTime"){
            /*
            Calculates the pose and heading from the data from the swerve modules; results in a Twist2d object.
             */
            val currentPositions = drivetrain.currentModulePositions // uses getter variable; thus, value must be fetched once
            wheelDeltas.apply{
                topLeftDistance = currentPositions.topLeftDistance - previousDistances[0]
                previousDistances[0] = currentPositions.topLeftDistance

                topRightDistance = currentPositions.topRightDistance - previousDistances[1]
                previousDistances[1] = currentPositions.topRightDistance

                bottomLeftDistance = currentPositions.bottomLeftDistance - previousDistances[2]
                previousDistances[2] = currentPositions.bottomLeftDistance

                bottomRightDistance = currentPositions.bottomRightDistance - previousDistances[3]
                previousDistances[3] = currentPositions.bottomRightDistance

                topLeftAngle = currentPositions.topLeftAngle
                topRightAngle = currentPositions.topRightAngle
                bottomLeftAngle = currentPositions.bottomLeftAngle
                bottomRightAngle = currentPositions.bottomRightAngle
            }
            val twist = drivetrain.kinematics.toTwist2d(*wheelDeltas.toArray())
            recordOutput("calculatedHeadingRad",poseEstimator.latestPose.rotation.radians + twist.dtheta)

            /*
            If a gyro is given, replace the calculated heading with the gyro's heading before adding the twist to the pose estimator.
             */
            if (drivetrain.gyro != null){
                val currentGyroHeading = drivetrain.gyro.heading
                twist.dtheta = (currentGyroHeading - lastGyroHeading).inUnit(radians)
                lastGyroHeading = currentGyroHeading
                recordOutput("Drivetrain(Swerve)/calculatedHeadingUsed", false)
            }else{
                recordOutput("Drivetrain(Swerve)/calculatedHeadingUsed", true)
            }
            poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds),twist)


            /*
            Sends all pose data to the pose estimator.
             */
            if (poseSuppliers.size > 0){
                visionUpdates.clear()
                poseSuppliers.forEach{
                    val measurement = it.robotPoseEstimate
                    if (measurement != null){
                        val stdDevVector = NomadApriltagUtil.calculateVisionUncertainty(
                            measurement.value.x.siValue,
                            heading.asRotation2d(),
                            it.cameraYaw.asRotation2d(),
                        )
                        visionUpdates.add(
                            TimestampedVisionUpdate(
                                measurement.timestamp.inUnit(seconds),
                                measurement.value.inUnit(meters),
                                stdDevVector
                            )
                        )
                    }
                }
                if (visionUpdates.size != 0) poseEstimator.addVisionData(visionUpdates)
            }

            /*
            Records the robot's pose on the field and in AdvantageScope.
             */
            field.robotPose = poseEstimator.latestPose
            recordOutput("Drivetrain(Swerve)/Pose2d", Pose2d.struct, poseEstimator.latestPose)
            recordOutput("Drivetrain(Swerve)/realGyroUsedInPoseEstimation", drivetrain.gyro != null)
            recordOutput("Drivetrain(Swerve)/realGyroHeadingRad", drivetrain.gyro?.heading?.inUnit(radians) ?: 0.0)
        }
    }
}
