@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.external.frc6995.NomadAprilTagUtil
import frc.external.limelight.LimelightHelpers
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy


/**
 * Represents a pose monitor which combines wheel odometry(and optionally vision data)
 * to produce a robot pose.
 *
 * This class should not be implemented by vision cameras or sensors, as their pose results
 * are not always accurate, and thus the nullable parameters are necessary.
 */
abstract class RobotPoseMonitor(name: String): HeadingProvider, SuperSubsystem(name) {
    abstract val robotPose: UnitPose2d

    abstract fun resetPose(pose: UnitPose2d = UnitPose2d())

    abstract fun addVisionMeasurement(measurement: Measurement<UnitPose2d>, stdDevs: Matrix<N3, N1>? = null)



    fun addVisionMeasurement(measurement: Measurement<UnitPose2d>, cameraYaw: Angle){
        addVisionMeasurement(
            measurement,
            NomadAprilTagUtil.calculateVisionUncertainty(
                measurement.value.x.inUnit(meters),
                measurement.value.rotation.asRotation2d(),
                cameraYaw.asRotation2d()
            )
        )
    }

    fun registerPhotonCamera(
        photonCam: PhotonCamera,
        robotToCamera: UnitTransform3d,
        poseStrategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    ){
        val poseEstimator = PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            poseStrategy,
            photonCam,
            robotToCamera.inUnit(meters)
        )
        val camName = photonCam.name

        ChargerRobot.runPeriodically {
            val poseEstimation = poseEstimator.update()
            if (poseEstimation.isPresent){
                log(Pose3d.struct, "PhotonPoseEstimations/$camName", poseEstimation.get().estimatedPose)
                addVisionMeasurement(
                    Measurement(
                        poseEstimation.get().estimatedPose.toPose2d().ofUnit(meters),
                        poseEstimation.get().timestampSeconds.ofUnit(seconds)
                    ),
                    poseEstimator.robotToCameraTransform.rotation.x.ofUnit(radians)
                )
            }else{
                log(Pose3d.struct, "PhotonPoseEstimations/$camName", Pose3d())
            }
        }
    }

    fun registerLimelight(
        camName: String,
        robotToCamera: UnitTransform3d,
        useMegaTag2: Boolean
    ){
        LimelightHelpers.setCameraPose_RobotSpace(
            camName,
            robotToCamera.x.inUnit(meters),
            robotToCamera.y.inUnit(meters),
            robotToCamera.z.inUnit(meters),
            robotToCamera.rotation.x,
            robotToCamera.rotation.y,
            robotToCamera.rotation.z
        )

        ChargerRobot.runPeriodically {
            val poseEstimation = if (useMegaTag2){
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName)
            }else{
                LimelightHelpers.getBotPoseEstimate_wpiBlue(camName)
            }

            if (poseEstimation.tagCount > 0){
                addVisionMeasurement(
                    Measurement(
                        poseEstimation.pose.ofUnit(meters),
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

    override val heading: Angle get() = robotPose.rotation
}