package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.external.frc6995.NomadAprilTagUtil
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
    companion object{
        private var logVisionlessPose: Boolean = true

        /**
         * Disables logging of the calculated pose without vision.
         */
        @Suppress("unused")
        fun disableVisionlessPoseLogging(){
            logVisionlessPose = false
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
        var gyroRotatingTooFast = false
        recordOutput(drivetrain.logName + "/realGyroUsedInPoseEstimation", drivetrain.gyro != null)

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

        val basePose = poseEstimator.update(
            gyroHeading.asRotation2d(),
            drivetrain.modulePositions.toTypedArray()
        )

        if (gyroRotatingTooFast){
            println("Vision pose readings ignored; gyro is rotating too fast.")
        }else{
            for (visionEstimator in visionEstimators){
                for (poseEstimate in visionEstimator.robotPoseEstimates){
                    if (poseEstimate.value.distanceTo(basePose.ofUnit(meters)) < 0.8.meters){
                        val visionStandardDeviationVector = NomadAprilTagUtil.calculateVisionUncertainty(
                            poseEstimate.value.x.inUnit(meters),
                            gyroHeading.asRotation2d(),
                            visionEstimator.cameraYaw.asRotation2d()
                        )

                        poseEstimator.addVisionMeasurement(
                            poseEstimate.value.inUnit(meters),
                            poseEstimate.timestamp.inUnit(seconds),
                            visionStandardDeviationVector
                        )
                    }
                }
            }
        }

        recordOutput(drivetrain.logName + "/Pose2d", Pose2d.struct, poseEstimator.estimatedPosition)
        robotObject.pose = poseEstimator.estimatedPosition

        if (logVisionlessPose && visionEstimators.size > 0){
            recordOutput(
                drivetrain.logName + "/Pose2dWithoutVisionEstimation",
                Pose2d.struct,
                noVisionPoseEstimator.update(
                    gyroHeading.asRotation2d(),
                    drivetrain.modulePositions.toTypedArray()
                )
            )
        }
    }
}