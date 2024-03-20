@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d

/**
 * A Helper class used to get the pose of an [EncoderDifferentialDrivetrain].
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
class DifferentialPoseMonitor(
    private val drivetrain: EncoderDifferentialDrivetrain,
    visionEstimators: List<VisionPoseSupplier>,
    startingPose: UnitPose2d
): SubsystemBase(), RobotPoseMonitor {
    private val poseEstimator = DifferentialDrivePoseEstimator(
        drivetrain.kinematics,
        Rotation2d(0.0),
        0.0, 0.0,
        Pose2d()
    )

    private val visionEstimators: MutableList<VisionPoseSupplier> = mutableListOf()


    init{
        addPoseSuppliers(*visionEstimators.toTypedArray())
        resetPose(startingPose)
    }


    override val robotPose: UnitPose2d
        get() = poseEstimator.estimatedPosition.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d) {
        if (drivetrain.gyro is ZeroableHeadingProvider){
            drivetrain.gyro.zeroHeading(pose.rotation)
        }

        poseEstimator.resetPosition(
            (drivetrain.gyro?.heading ?: pose.rotation).asRotation2d(),
            drivetrain.leftWheelTravel.siValue * drivetrain.wheelTravelPerMotorRadian.siValue,
            drivetrain.rightWheelTravel.siValue * drivetrain.wheelTravelPerMotorRadian.siValue,
            pose.inUnit(meters)
        )
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {
        visionEstimators.addAll(visionSystems)
    }


    override fun periodic(){
        poseEstimator.update(
            (drivetrain.gyro?.heading ?: drivetrain.heading).asRotation2d(),
            drivetrain.leftWheelTravel.siValue * drivetrain.wheelTravelPerMotorRadian.siValue,
            drivetrain.rightWheelTravel.siValue * drivetrain.wheelTravelPerMotorRadian.siValue,
        )

        for (visionEstimator in visionEstimators){
            for (visionEstimate in visionEstimator.robotPoseEstimates){
                poseEstimator.addVisionMeasurement(
                    visionEstimate.value.inUnit(meters),
                    visionEstimate.timestamp.inUnit(seconds)
                )
            }
        }

        ChargerRobot.FIELD.robotPose = poseEstimator.estimatedPosition
    }
}