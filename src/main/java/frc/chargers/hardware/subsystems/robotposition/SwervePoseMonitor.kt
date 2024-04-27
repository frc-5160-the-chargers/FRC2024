package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d


/**
 * A Helper class used to get the pose of an [EncoderHolonomicDrivetrain].
 *
 * Most of the time, you will not need to instantiate this class directly;
 * instead, call drivetrainInstance.poseEstimator to access the built-in pose estimator
 * of the drivetrain.
 */
class SwervePoseMonitor(
    private val drivetrain: EncoderHolonomicDrivetrain,
    startingPose: UnitPose2d
): RobotPoseMonitor(drivetrain.namespace + "/PoseMonitor") {
    companion object{
        private var logVisionlessPose: Boolean = false

        /**
         * Disables logging of the calculated pose without vision.
         */
        @Suppress("unused")
        fun enableVisionlessPoseLogging(){
            logVisionlessPose = true
        }
    }

    // the wpilib pose estimator that powers the pose monitor.
    private val poseEstimator = SwerveDrivePoseEstimator(
        drivetrain.kinematics,
        Rotation2d(0.0),
        drivetrain.modulePositions.toTypedArray(),
        Pose2d()
    )

    // purely for logging/comparison purposes for debugging
    private val noVisionPoseEstimator = SwerveDriveOdometry(
        drivetrain.kinematics,
        Rotation2d(0.0),
        drivetrain.modulePositions.toTypedArray(),
        Pose2d()
    )

    private var gyroHeading = Angle(0.0)

    private var gyroRotatingTooFast = false

    private val robotObject = ChargerRobot.FIELD.getObject(drivetrain.namespace)

    private var previousWheelTravelDistances: MutableList<Distance> =
        drivetrain.modulePositions.map{ it.distanceMeters.ofUnit(meters) }.toMutableList()


    init{
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

    override fun addVisionMeasurement(measurement: Measurement<UnitPose2d>, stdDevs: Matrix<N3, N1>?) {
        if (gyroRotatingTooFast){
            println("Vision pose readings ignored; gyro is rotating too fast.")
        }else{
            if (stdDevs != null){
                poseEstimator.addVisionMeasurement(
                    measurement.value.inUnit(meters),
                    measurement.timestamp.inUnit(seconds),
                    stdDevs
                )
            }else{
                poseEstimator.addVisionMeasurement(
                    measurement.value.inUnit(meters),
                    measurement.timestamp.inUnit(seconds)
                )
            }
        }
    }

    override fun periodic(){
        log("RealGyroUsedInPoseEstimation", drivetrain.gyro != null)

        if (drivetrain.gyro != null){
            val newHeading = drivetrain.gyro.heading
            gyroRotatingTooFast = abs(newHeading - gyroHeading) / ChargerRobot.LOOP_PERIOD > 720.degrees / 1.seconds
            gyroHeading = newHeading
        }else{
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

        log(Pose2d.struct, "Pose2d", poseEstimator.estimatedPosition)
        robotObject.pose = poseEstimator.estimatedPosition

        if (logVisionlessPose){
            log(
                Pose2d.struct,
                "Pose2dWithoutVisionEstimation",
                noVisionPoseEstimator.update(
                    gyroHeading.asRotation2d(),
                    drivetrain.modulePositions.toTypedArray()
                )
            )
        }
    }
}