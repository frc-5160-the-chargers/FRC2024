package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import org.littletonrobotics.junction.Logger.recordOutput


/**
 * A Helper class used to get the pose of an [EncoderHolonomicDrivetrain].
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
class SwervePoseMonitor(
    private val drivetrain: EncoderHolonomicDrivetrain,
    visionEstimators: List<VisionPoseSupplier>,
    startingPose: UnitPose2d
): SubsystemBase(), RobotPoseMonitor {
    private val poseEstimator = SwerveDrivePoseEstimator(
        drivetrain.kinematics,
        Rotation2d(0.0),
        drivetrain.modulePositions.toTypedArray(),
        Pose2d()
    )

    private var gyroHeading = Angle(0.0)

    private val visionEstimators: MutableList<VisionPoseSupplier> = mutableListOf()

    private val robotObject = ChargerRobot.FIELD.getObject(drivetrain.logName)

    private var previousWheelTravelDistances: MutableList<Distance> =
        drivetrain.modulePositions.map{ it.distanceMeters.ofUnit(meters) }.toMutableList()


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
        // here, we do not take the gyro heading directly.
        // If we do, we will still be reading the old gyro heading value,
        // as the new(zeroed) value will not be updated until the next loop.
        // In addition, the gyro will be zeroed to the pose's rotation next loop anyways.
        gyroHeading = pose.rotation
        poseEstimator.resetPosition(
            gyroHeading.asRotation2d(),
            drivetrain.modulePositions.toTypedArray(),
            pose.inUnit(meters)
        )
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {
        visionEstimators.addAll(visionSystems)
    }


    override fun periodic(){
        if (drivetrain.gyro != null){
            recordOutput(drivetrain.logName + "/realGyroUsedInPoseEstimation", true)
            gyroHeading = drivetrain.gyro.heading
        }else{
            recordOutput(drivetrain.logName + "/realGyroUsedInPoseEstimation", false)
            // wheelDeltas represent the difference in position moved during the loop,
            // as well as the current angle(not the change in angle).
            val wheelDeltas = drivetrain.modulePositions.mapIndexed{ i, originalPosition ->
                val delta = originalPosition.distanceMeters - previousWheelTravelDistances[i].inUnit(meters)
                previousWheelTravelDistances[i] = originalPosition.distanceMeters.ofUnit(meters)
                SwerveModulePosition(delta, originalPosition.angle)
            }

            gyroHeading += drivetrain
                .kinematics
                .toTwist2d(*wheelDeltas.toTypedArray())
                .dtheta
                .ofUnit(radians)
        }

        poseEstimator.update(
            gyroHeading.asRotation2d(),
            drivetrain.modulePositions.toTypedArray()
        )

        for (visionEstimator in visionEstimators){
            for (visionEstimate in visionEstimator.robotPoseEstimates){
                poseEstimator.addVisionMeasurement(
                    visionEstimate.value.inUnit(meters),
                    visionEstimate.timestamp.inUnit(seconds)
                )
            }
        }

        recordOutput(drivetrain.logName + "/Pose2d", Pose2d.struct, poseEstimator.estimatedPosition)
        robotObject.pose = poseEstimator.estimatedPosition
    }
}