@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision.photonvision.simulation

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.vision.VisionCameraConstants
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.sensors.vision.photonvision.toAprilTagTarget
import frc.chargers.hardware.sensors.vision.photonvision.toObjectTarget
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.*
import kotlin.collections.LinkedHashSet

private val APRILTAG_FIELD_SIM = VisionSystemSim("AprilTags").apply{
    addAprilTags(ChargerRobot.APRILTAG_LAYOUT)
}

private var cam_counter = 1


/**
 * Represents a Vision Camera within simulation.
 */
class VisionCameraSim(
    private val poseEstimator: RobotPoseMonitor,
    private val getRobotToCamera: () -> UnitTransform3d,
    private val simCameraProperties: SimCameraProperties = SimCameraProperties(),
): SubsystemBase() {
    private val allVisionSystems: LinkedHashSet<VisionSystemSim> = linkedSetOf()

    private var previousRobotToCam = getRobotToCamera()


    inner class AprilTagPipeline(
        /**
         * The namespace of which to log to.
         *
         * Keep this the same between real and sim variants!
         */
        logInputs: LoggableInputsProvider
    ): SimVisionPipelineBase<VisionTarget.AprilTag>(APRILTAG_FIELD_SIM){
        override val visionTargets: List<VisionTarget.AprilTag>
            by logInputs.valueList(default = VisionTarget.AprilTag.Dummy){
                val result = CAM_SIM.camera.latestResult

                if (!result.hasTargets()) return@valueList listOf()

                return@valueList result
                    .getTargets()
                    // see VisionTargetConversion.kt
                    .map{ target -> toAprilTagTarget(target, result.timestampSeconds.ofUnit(seconds)) }
            }
    }

    inner class ObjectPipeline(
        /**
         * The namespace of which to log to.
         *
         * Keep this the same between real and sim variants!
         */
        logInputs: LoggableInputsProvider,
        visionSystemSim: VisionSystemSim
    ): SimVisionPipelineBase<VisionTarget.Object>(visionSystemSim){
        override val visionTargets: List<VisionTarget.Object>
            by logInputs.valueList(default = VisionTarget.Object.Dummy){
                val result = CAM_SIM.camera.latestResult

                if (!result.hasTargets()) return@valueList listOf()

                return@valueList result
                    .getTargets()
                    .map{ target -> toObjectTarget(target, result.timestampSeconds.ofUnit(seconds)) }
            }
    }
    
    
    inner class AprilTagPoseEstimator(
        logInputs: LoggableInputsProvider,
        strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    ): PhotonPoseEstimator(
        ChargerRobot.APRILTAG_LAYOUT,
        strategy,
        PhotonCamera("Sim Camera pipeline #$cam_counter"),
        previousRobotToCam.inUnit(meters)
    ), VisionPoseSupplier {
        
        override val cameraYaw: Angle
            get() = Angle(previousRobotToCam.rotation.z)

        init{
            setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        }

        override val robotPoseEstimates: List<Measurement<UnitPose2d>>
            by logInputs.valueList(default = Measurement(UnitPose2d(), Time(0.0))){
                when(val signal = update()){
                    Optional.empty<EstimatedRobotPose>() -> listOf()

                    else -> listOf(
                        Measurement(
                            value = UnitPose2d(signal.get().estimatedPose.toPose2d()),
                            timestamp = signal.get().timestampSeconds.ofUnit(seconds)
                        )
                    )
                }
            }
    }


    abstract inner class SimVisionPipelineBase <T: VisionTarget>(
        visionSystemSim: VisionSystemSim,
    ): VisionPipeline<T> {

        private val cameraName = "Sim Camera Pipeline #$cam_counter".also{ cam_counter++ }

        // every single pipeline has its own camera
        // due to the way visionsystemsim works
        protected val CAM_SIM = PhotonCameraSim(
            PhotonCamera(cameraName),
            simCameraProperties
        )

        init{
            visionSystemSim.addCamera(CAM_SIM, previousRobotToCam.inUnit(meters))
            allVisionSystems.add(visionSystemSim)
        }

        override val cameraConstants: VisionCameraConstants
            get() = VisionCameraConstants(
                cameraName,
                previousRobotToCam.z,
                previousRobotToCam.rotation.y.ofUnit(radians)
            )

        override fun reset() {
            println("Sim camera pipeline has been reset.")
        }
    }

    override fun periodic(){
        // updates robot to camera periodically
        val currentRobotToCam = getRobotToCamera()

        // updates vision systems and adjusts camera poses
        allVisionSystems.forEach{ visionSystemSim ->
            visionSystemSim.update(poseEstimator.robotPose.inUnit(meters))

            if (currentRobotToCam != previousRobotToCam){
                visionSystemSim.cameraSims.forEach{camSim ->
                    visionSystemSim.adjustCamera(camSim, currentRobotToCam.inUnit(meters))
                }
            }
        }

        previousRobotToCam = currentRobotToCam
    }


}