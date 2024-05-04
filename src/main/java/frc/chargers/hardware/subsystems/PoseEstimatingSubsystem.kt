package frc.chargers.hardware.subsystems

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.SuperSubsystem
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.Rotation2d
import frc.external.frc6995.NomadAprilTagUtil
import frc.external.limelight.LimelightHelpers
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy

/**
 * A base class for a subsystem that can estimate its own pose,
 * and fuse vision pose measurements.
 *
 * This is usually a drivetrain.
 */
@Suppress("unused")
abstract class PoseEstimatingSubsystem(namespace: String): SuperSubsystem(namespace) {
    abstract val robotPose: Pose2d

    abstract fun resetPose(pose: Pose2d = Pose2d())

    abstract fun addVisionMeasurement(measurement: Measurement<Pose2d>, stdDevs: Matrix<N3, N1>? = null)


    /**
     * Adds a vision pose measurement; calculating pose standard deviation from camera yaw.
     */
    fun addVisionMeasurement(measurement: Measurement<Pose2d>, cameraYaw: Angle){
        addVisionMeasurement(
            measurement,
            NomadAprilTagUtil.calculateVisionUncertainty(
                measurement.value.x,
                measurement.value.rotation,
                Rotation2d(cameraYaw)
            )
        )
    }

    /**
     * Registers a photon camera for vision pose estimation.
     */
    fun addPhotonPoseSupplier(
        photonCam: PhotonCamera,
        robotToCamera: Transform3d,
        poseStrategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    ){
        val poseEstimator = PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            poseStrategy,
            photonCam,
            robotToCamera
        )
        val camName = photonCam.name

        ChargerRobot.runPeriodic {
            val poseEstimation = poseEstimator.update()
            if (poseEstimation.isPresent){
                log(Pose3d.struct, "PhotonPoseEstimations/$camName", poseEstimation.get().estimatedPose)
                addVisionMeasurement(
                    Measurement(
                        poseEstimation.get().estimatedPose.toPose2d(),
                        poseEstimation.get().timestampSeconds.ofUnit(seconds)
                    ),
                    poseEstimator.robotToCameraTransform.rotation.x.ofUnit(radians)
                )
            }else{
                log(Pose3d.struct, "PhotonPoseEstimations/$camName", Pose3d())
            }
        }
    }

    /**
     * Registers a limelight for vision pose estimation.
     *
     * Precondition: If useMegaTag2 is enabled, you must call LimelightHelpers.setRobotOrientation periodically.
     * This can be accomplished via gyro.broadcastOrientationForMegaTag2(),
     * where the gyro can be a [frc.chargers.hardware.sensors.imu.ChargerNavX] or a [frc.chargers.hardware.sensors.imu.ChargerPigeon2].
     */
    fun addLimelightPoseSupplier(
        camName: String,
        robotToCamera: Transform3d,
        useMegaTag2: Boolean
    ){
        LimelightHelpers.setCameraPose_RobotSpace(
            camName,
            robotToCamera.x,
            robotToCamera.y,
            robotToCamera.z,
            robotToCamera.rotation.x,
            robotToCamera.rotation.y,
            robotToCamera.rotation.z
        )

        ChargerRobot.runPeriodic {
            val poseEstimation = if (useMegaTag2){
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName)
            }else{
                LimelightHelpers.getBotPoseEstimate_wpiBlue(camName)
            }

            val tagEstimateAmbiguous = poseEstimation.rawFiducials.size == 1 &&
                poseEstimation.rawFiducials[0].ambiguity >= 0.9

            if (poseEstimation.tagCount > 0 && !tagEstimateAmbiguous){
                addVisionMeasurement(
                    Measurement(
                        poseEstimation.pose,
                        poseEstimation.timestampSeconds.ofUnit(seconds)
                    ),
                    robotToCamera.rotation.z.ofUnit(radians)
                )
                log(Pose2d.struct, "LimelightPoseEstimations/$camName", poseEstimation.pose)
            }else{
                log(Pose2d.struct, "LimelightPoseEstimations/$camName", Pose2d())
            }
        }
    }
}