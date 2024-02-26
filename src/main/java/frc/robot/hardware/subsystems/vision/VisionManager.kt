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

class VisionManager(poseEstimator: RobotPoseMonitor, tunableCamerasInSim: Boolean = false): SubsystemBase() {
    private val noteTargetModel = TargetModel(14.inches, 14.inches, 2.inches) // custom overload

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
    private val robotToMLWebcam = UnitTransform3d(
        UnitTranslation3d(x = 0.meters, y = 0.meters, z = 10.inches),
        Rotation3d(roll = 0.degrees, pitch = 30.degrees, yaw = 180.degrees)
    ) // tbd atm

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
            val photonWebcamSim = VisionCameraSim(poseEstimator, robotToMLWebcam, SimCameraProperties())

            if (tunableCamerasInSim){
                val dashTuner = DashboardTuner("SimVisionTuning")
                val tunableLLTransform by dashTuner.transform3d(robotToLimelight)
                val tunableArducamTransform by dashTuner.transform3d(robotToArducam)
                val tunableWebcamTransform by dashTuner.transform3d(robotToMLWebcam)

                ChargerRobot.runPeriodically{
                    //limelightSim.updateRobotToCamera(tunableLLTransform)
                    //photonArducamSim.updateRobotToCamera(tunableArducamTransform)
                    photonWebcamSim.updateRobotToCamera(tunableWebcamTransform)
                }
            }

            leftTagPipeline = limelightSim.AprilTagPipeline(leftCamLog)
                .also{ poseSources.add(it) }

            rightTagPipeline = photonArducamSim.AprilTagPipeline(rightCamLog)
                .also{ poseSources.add(it) }

            notePipeline = photonWebcamSim.ObjectPipeline(noteDetectorLog, mlTargetField)
        }

        poseEstimator.addPoseSuppliers(*poseSources.toTypedArray())
        addNotes(
            UnitPose2d(8.3.meters, 7.45.meters, 0.degrees),
            UnitPose2d(8.3.meters, 5.77.meters, 0.degrees),
            UnitPose2d(8.3.meters, 4.13.meters, 0.degrees),
            UnitPose2d(2.9.meters, 7.05.meters, 0.degrees)
        )
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
        Logger.recordOutput("MLCam/robotToTargetDistance", notePipeline.robotToTargetDistance(targetHeight = 0.inches)?.siValue ?: 0.0)
    }
}