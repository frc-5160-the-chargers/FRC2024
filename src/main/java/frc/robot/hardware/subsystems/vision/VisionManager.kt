@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.vision

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.FusedAprilTagPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
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
import org.photonvision.simulation.VisionTargetSim

class VisionManager(poseEstimator: RobotPoseMonitor): SubsystemBase() {
    private val noteTargetModel = TargetModel(2.inches, 14.inches) // custom overload

    // handles logging & replay for vision cameras
    private val leftCamLog = LoggableInputsProvider("AprilTagCamLeft")
    private val rightCamLog = LoggableInputsProvider("AprilTagCamRight")
    private val noteDetectorLog = LoggableInputsProvider("ObjectDetector")

    // transforms for vision cameras
    private val robotToLimelight = UnitTransform3d(
        UnitTranslation3d(x = 0.meters, y = 0.meters, z = 20.inches),
        Rotation3d(roll = 0.degrees, pitch = (-45).degrees, yaw = 0.degrees)
    ) // tbd atm
    private val robotToArducam = robotToLimelight // tbd atm
    private val robotToMLWebcam = UnitTransform3d() // tbd atm

    private val mlTargetField = VisionSystemSim("ML Vision System")



    // all of these are automatically akit loggable and replayable
    // via LoggableInputsProvider

    val leftTagPipeline: AprilTagVisionPipeline // apriltag pipeline on left(arducam from orange pi)
    val rightTagPipeline: AprilTagVisionPipeline // apriltag pipeline on right(limelight 2)
    val notePipeline: ObjectVisionPipeline // object pipeline(webcam from orange pi)

    init{
        val poseSources: MutableList<VisionPoseSupplier> = mutableListOf()

        if (RobotBase.isReal()){
            val limelight = ChargerLimelight(useJsonDump = false, robotToCamera = robotToLimelight)
            val photonArducam = ChargerPhotonCamera(name = "AprilTag Arducam", robotToCamera = robotToArducam)
            val photonWebcam = ChargerPhotonCamera(name = "ML Webcam", robotToCamera = robotToMLWebcam)

            leftTagPipeline = limelight.AprilTagPipeline(index = 0, leftCamLog, usePoseEstimation = true)
                .also{ poseSources.add(it) }

            rightTagPipeline = photonArducam.AprilTagPipeline(index = 0, rightCamLog, usePoseEstimation = true)
                .also{ poseSources.add(it) }

            notePipeline = photonWebcam.ObjectPipeline(index = 0, noteDetectorLog)
        }else{
            val limelightSim = VisionCameraSim(poseEstimator, robotToLimelight, SimCameraProperties.LL2_640_480())
            val photonArducamSim = VisionCameraSim(poseEstimator, robotToArducam, SimCameraProperties.PI4_LIFECAM_640_480())
            val photonWebcamSim = VisionCameraSim(poseEstimator, robotToMLWebcam, SimCameraProperties.PI4_LIFECAM_640_480())

            val mlTargetField = VisionSystemSim("ML Vision System")

            leftTagPipeline = limelightSim.AprilTagPipeline(leftCamLog)
                .also{ poseSources.add(it) }

            rightTagPipeline = photonArducamSim.AprilTagPipeline(rightCamLog)
                .also{ poseSources.add(it) }

            notePipeline = photonWebcamSim.ObjectPipeline(noteDetectorLog, mlTargetField)
        }

        poseEstimator.addPoseSuppliers(*poseSources.toTypedArray())
    }

    // fuses the left and right apriltag pipelines together
    val fusedTagPipeline: AprilTagVisionPipeline =
        FusedAprilTagPipeline(shouldAverageBestTargets = false, leftTagPipeline, rightTagPipeline)

    fun addSimVisionTargets(vararg visionTargets: VisionTargetSim){
        mlTargetField.addVisionTargets(*visionTargets)
    }

    fun addSimVisionTargets(type: String, vararg visionTargets: VisionTargetSim){
        mlTargetField.addVisionTargets(type, *visionTargets)
    }

    fun addNotes(vararg poses: UnitPose3d){
        addSimVisionTargets("Note", *poses.map{ VisionTargetSim(it, noteTargetModel) }.toTypedArray())
    }

    fun addNotes(vararg poses: UnitPose2d){
        addNotes(*poses.map{ it.toPose3d() }.toTypedArray())
    }

    override fun periodic(){
        Logger.recordOutput("AprilTagCam/robotToTargetDistance", fusedTagPipeline.robotToTargetDistance(targetHeight = 1.35582.meters)?.siValue ?: 0.0)
    }
}