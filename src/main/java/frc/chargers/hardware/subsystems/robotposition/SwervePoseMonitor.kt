@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.external.frc6328.MechanicalAdvantagePoseEstimator
import frc.external.frc6328.MechanicalAdvantagePoseEstimator.TimestampedVisionUpdate
import frc.external.frc6995.NomadApriltagUtil
import frc.chargers.advantagekitextensions.recordLatency
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
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
    private val visionPoseSuppliers: MutableList<VisionPoseSupplier>,
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseMonitor {

    /* Public API */
    public constructor(
        drivetrain: EncoderHolonomicDrivetrain,
        startingPose: UnitPose2d = UnitPose2d(),
        vararg poseSuppliers: VisionPoseSupplier,
    ): this(
        drivetrain,
        poseSuppliers.toMutableList(),
        startingPose
    )

    override val robotPose: UnitPose2d get() = poseEstimator.latestPose.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
        if (drivetrain.gyro is ZeroableHeadingProvider){
            drivetrain.gyro.zeroHeading(pose.rotation)
        }else if (drivetrain.gyro == null){
            println(poseEstimator.latestPose)
        }
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier){
        this.visionPoseSuppliers.addAll(visionSystems)
    }

    /* Private Implementation */
    private val poseEstimator = MechanicalAdvantagePoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001),
    ).also{ it.resetPose(startingPose.inUnit(meters)) }

    // Array used because members should be mutable, but adding members should not be allowed
    private val previousDistances: Array<Distance> = drivetrain.modulePositions.map{ it.distanceMeters.ofUnit(meters) }.toTypedArray()
    private var lastGyroHeading = Angle(0.0)
    private var wheelDeltas = List(4){ Distance(0.0) }
    private val visionUpdates: MutableList<TimestampedVisionUpdate> = mutableListOf()

    override fun periodic(){
        recordLatency("SwervePoseMonitorLoopTime"){
            /*
            Calculates the pose and heading from the data from the swerve modules; results in a Twist2d object.
             */
            val currentPositions = drivetrain.modulePositions // uses getter variable; thus, value must be fetched once

            wheelDeltas = currentPositions.mapIndexed{ i, modPosition ->
                val currentDistance = modPosition.distanceMeters.ofUnit(meters)
                val delta = currentDistance - previousDistances[i]
                previousDistances[i] = currentDistance
                return@mapIndexed delta
            }

            val twist = drivetrain.kinematics.toTwist2d(
                *Array(4){ index ->
                    SwerveModulePosition(wheelDeltas[index].siValue, currentPositions[index].angle)
                }
            )

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
            poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds), twist)


            /*
            Sends all pose data to the pose estimator.
             */
            visionUpdates.clear()

            for (visionPoseSupplier in visionPoseSuppliers){
                for (poseEstimate in visionPoseSupplier.robotPoseEstimates){
                    val stdDevVector = NomadApriltagUtil.calculateVisionUncertainty(
                        poseEstimate.value.x.siValue,
                        heading.asRotation2d(),
                        visionPoseSupplier.cameraYaw.asRotation2d(),
                    )

                    visionUpdates.add(
                        TimestampedVisionUpdate(
                            poseEstimate.timestamp.inUnit(seconds),
                            poseEstimate.value.inUnit(meters),
                            stdDevVector
                        )
                    )
                }
            }

            if (visionUpdates.size != 0) poseEstimator.addVisionData(visionUpdates)

            /*
            Records the robot's pose on the field and in AdvantageScope.
             */
            ChargerRobot.FIELD.robotPose = poseEstimator.latestPose
            recordOutput("Drivetrain(Swerve)/Pose2d", Pose2d.struct, poseEstimator.latestPose)
            recordOutput("Drivetrain(Swerve)/realGyroUsedInPoseEstimation", drivetrain.gyro != null)
            recordOutput("Drivetrain(Swerve)/realGyroHeadingRad", drivetrain.gyro?.heading?.inUnit(radians) ?: 0.0)
        }
    }
}
