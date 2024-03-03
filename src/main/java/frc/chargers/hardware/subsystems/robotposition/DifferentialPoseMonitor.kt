@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.ChargerRobot
import frc.external.frc6328.MechanicalAdvantagePoseEstimator
import frc.external.frc6328.MechanicalAdvantagePoseEstimator.TimestampedVisionUpdate
import frc.external.frc6995.NomadAprilTagUtil
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import org.littletonrobotics.junction.Logger.recordOutput

/**
 * A Helper class used to get the pose of an [EncoderDifferentialDrivetrain].
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
public class DifferentialPoseMonitor(
    private val drivetrain: EncoderDifferentialDrivetrain,
    private val visionPoseSuppliers: MutableList<VisionPoseSupplier> = mutableListOf(),
    startingPose: UnitPose2d = UnitPose2d()
): SubsystemBase(), RobotPoseMonitor {
    public constructor(
        drivetrain: EncoderDifferentialDrivetrain,
        startingPose: UnitPose2d = UnitPose2d(),
        vararg poseSuppliers: VisionPoseSupplier,
    ): this(
        drivetrain,
        poseSuppliers.toMutableList(),
        startingPose
    )

    override val robotPose: UnitPose2d get() = poseEstimator.latestPose.ofUnit(meters)

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {
        visionPoseSuppliers.addAll(visionSystems)
    }

    override fun resetPose(pose: UnitPose2d){
        poseEstimator.resetPose(pose.inUnit(meters))
        if (drivetrain.gyro is ZeroableHeadingProvider){
            drivetrain.gyro.zeroHeading(pose.rotation)
        }
    }



    /* Private Implementation */
    private val poseEstimator = MechanicalAdvantagePoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001)
    ).also{ it.resetPose(startingPose.inUnit(meters)) }
    private var previousDistanceL = drivetrain.leftWheelTravel * drivetrain.wheelTravelPerMotorRadian
    private var previousDistanceR = drivetrain.leftWheelTravel * drivetrain.wheelTravelPerMotorRadian
    private var previousGyroHeading = drivetrain.gyro?.heading ?: Angle(0.0)
    private val visionUpdates: MutableList<TimestampedVisionUpdate> = mutableListOf()


    override fun periodic(){
        val distanceL = drivetrain.leftWheelTravel * drivetrain.wheelTravelPerMotorRadian
        val distanceR = drivetrain.leftWheelTravel * drivetrain.wheelTravelPerMotorRadian

        val twist = drivetrain.kinematics.toTwist2d(
            (distanceL-previousDistanceL).inUnit(meters),
            (distanceR-previousDistanceR).inUnit(meters)
        )

        previousDistanceL = distanceL
        previousDistanceR = distanceR

        recordOutput("Drivetrain(Differential)/calculatedHeadingRad", poseEstimator.latestPose.rotation.rotations + twist.dtheta)

        if (drivetrain.gyro != null){
            val currentHeading = drivetrain.gyro.heading
            twist.dtheta = (currentHeading - previousGyroHeading).inUnit(radians)
            previousGyroHeading = currentHeading
            recordOutput("Drivetrain(Differential)/calculatedHeadingUsed", false)
        }else{
            recordOutput("Drivetrain(Differential)/calculatedHeadingUsed", false)
        }

        poseEstimator.addDriveData(fpgaTimestamp().inUnit(seconds),twist)

        visionUpdates.clear()

        for (visionPoseSupplier in visionPoseSuppliers){
            for (poseEstimate in visionPoseSupplier.robotPoseEstimates){
                val poseDelta = poseEstimate.value - robotPose
                if (poseDelta.translation.norm > 1.meters || abs(poseDelta.rotation) > 90.degrees){
                    println("Pose reading ignored!")
                    continue
                }

                val stdDevVector = NomadAprilTagUtil.calculateVisionUncertainty(
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


        ChargerRobot.FIELD.robotPose = poseEstimator.latestPose
        recordOutput("Drivetrain(Differential)/Pose2d", Pose2d.struct, poseEstimator.latestPose)
        recordOutput("Drivetrain(Differential)/realGyroUsedInPoseEstimation", drivetrain.gyro != null)
        recordOutput("Drivetrain(Differential)/realGyroHeadingRad", drivetrain.gyro?.heading?.inUnit(radians) ?: 0.0)
    }
}