@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.vision

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.DashboardTuner
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.sensors.vision.photonvision.ChargerPhotonCamera
import frc.chargers.hardware.sensors.vision.photonvision.simulation.TargetModel
import frc.chargers.hardware.sensors.vision.photonvision.simulation.VisionCameraSim
import frc.chargers.hardware.sensors.vision.photonvision.simulation.VisionTargetSim
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.wpilibextensions.geometry.threedimensional.Rotation3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTranslation3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.littletonrobotics.junction.Logger
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

class VisionManager(private val poseEstimator: RobotPoseMonitor, tunableCamerasInSim: Boolean = false): SubsystemBase() {
    private val noteTargetModel = TargetModel(14.inches, 14.inches, 2.inches) // custom overload

    // handles logging & replay for vision cameras
    private val aprilTagCamLog = LoggableInputsProvider("AprilTagArducam")
    private val noteDetectorLog = LoggableInputsProvider("ObjectDetector")

    // transforms for vision cameras
    private val robotToArducam = if (RobotBase.isReal()){
        // From ground intake POV
        // Alert alert: these numbers are kinda suspicious
        // rotation 3d not right yet
        // 30 30
        UnitTransform3d(
            // 42 inches up, 2 inches forward from center, 10.5 inches left from center
            UnitTranslation3d(x = 10.5.inches, y = 2.meters, z = 45.inches),
            // 30 degrees up, 30 degrees to the right
            Rotation3d(roll = -30.degrees, pitch = 30.degrees, yaw = 180.degrees)
        )
    }else{
        UnitTransform3d(
            UnitTranslation3d(x = 0.meters, y = 0.meters, z = 20.inches),
            Rotation3d(roll = 0.degrees, pitch = (-45).degrees, yaw = 180.degrees)
        )
    }
    private val robotToMLWebcam = UnitTransform3d(
        UnitTranslation3d(x = 0.meters, y = 0.meters, z = 10.inches),
        Rotation3d(roll = 0.degrees, pitch = 37.degrees, yaw = 0.degrees)
    ) // tbd atm

    private val mlTargetField = VisionSystemSim("ML Vision System")
    private val poseSources: MutableList<VisionPoseSupplier> = mutableListOf()



    // all of these are automatically akit loggable and replayable
    // via LoggableInputsProvider
    private val aprilTagPipeline: AprilTagVisionPipeline // apriltag pipeline on right(arducam from orange pi)
    val notePipeline: ObjectVisionPipeline // object pipeline(webcam from orange pi)

    init{
        if (RobotBase.isReal()){
            val photonArducam = ChargerPhotonCamera(name = "AprilTag Arducam", robotToCamera = robotToArducam)
            val photonWebcam = ChargerPhotonCamera(name = "ML Webcam", robotToCamera = robotToMLWebcam)

            aprilTagPipeline = photonArducam.AprilTagPipeline(index = 1, aprilTagCamLog, usePoseEstimation = true)
                .also{ poseSources.add(it) }

            notePipeline = photonWebcam.ObjectPipeline(index = 0, noteDetectorLog)

            aprilTagPipeline.reset()
            notePipeline.reset()
        }else{
            val photonArducamSim = VisionCameraSim(poseEstimator, robotToArducam, SimCameraProperties.PI4_LIFECAM_640_480())
            val photonWebcamSim = VisionCameraSim(poseEstimator, robotToMLWebcam, SimCameraProperties())

            if (tunableCamerasInSim){
                val dashTuner = DashboardTuner("SimVisionTuning")
                val tunableArducamTransform by dashTuner.transform3d(robotToArducam)
                val tunableWebcamTransform by dashTuner.transform3d(robotToMLWebcam)

                ChargerRobot.runPeriodically{
                    photonArducamSim.updateRobotToCamera(tunableArducamTransform)
                    photonWebcamSim.updateRobotToCamera(tunableWebcamTransform)
                }
            }

            aprilTagPipeline = photonArducamSim.AprilTagPipeline(aprilTagCamLog)
                .also{ poseSources.add(it) }

            notePipeline = photonWebcamSim.ObjectPipeline(noteDetectorLog, mlTargetField)
        }

        addNotes(
            UnitPose2d(8.3.meters, 7.45.meters, 0.degrees),
            UnitPose2d(8.3.meters, 5.77.meters, 0.degrees),
            UnitPose2d(8.3.meters, 4.13.meters, 0.degrees),
            UnitPose2d(2.9.meters, 7.05.meters, 0.degrees)
        )
    }

    // fuses the left and right apriltag pipelines together
    val tagPipeline: AprilTagVisionPipeline = aprilTagPipeline // tbd for now

    fun enableVisionPoseEstimation(){
        poseEstimator.addPoseSuppliers(*poseSources.toTypedArray())
    }

    fun addNotes(vararg poses: UnitPose3d){
        mlTargetField.addVisionTargets("Note", *poses.map{ VisionTargetSim(it, noteTargetModel) }.toTypedArray())
    }

    fun addNotes(vararg poses: UnitPose2d){
        addNotes(*poses.map{ it.toPose3d() }.toTypedArray())
    }

    override fun periodic(){
        Logger.recordOutput("PhotonCamera(AprilTagArducam)/robotToTargetDistance", tagPipeline.robotToTargetDistance(targetHeight = 1.35582.meters)?.siValue ?: 0.0)
        Logger.recordOutput("PhotonCamera(MLWebcam)/robotToTargetDistance", notePipeline.robotToTargetDistance(targetHeight = 0.inches)?.siValue ?: 0.0)
    }
}