@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision.photonvision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.vision.VisionCameraConstants
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.*

/**
 * A wrapper over PhotonVision's [PhotonCamera], with support for AdvantageKit and more integration within ChargerLib.
 *
 * Note: The robot transform to camera is represented like this:
 *
 * x is forward, y is left, z is up.
 * Rotation: x is roll, y is pitch, z is yaw.
 */
public class ChargerPhotonCamera(
    name: String,
    public val robotToCamera: UnitTransform3d
): PhotonCamera(name) {
    private var required: Boolean = false

    private val allIndexes: MutableList<Int> = mutableListOf()

    private fun ensureIndexValid(index: Int){
        require (index !in allIndexes){ "There is already a PhotonVision pipeline with index $index." }
        allIndexes.add(index)
    }


    init{
        ChargerRobot.runPeriodically{
            Logger.recordOutput("PhotonCamera($name)/pipelineIndex", pipelineIndex)
        }
    }

    /**
     * Represents a photonvision pipeline that can detect AprilTags,
     * and report the pose of the robot.
     */
    public inner class AprilTagPipeline(
        index: Int,
        /**
         * The namespace of which the Photon Camera Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider,
        disablePoseEstimation: Boolean,
        poseEstimationStrategy: PhotonPoseEstimator.PoseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    ): PhotonCameraPipeline<VisionTarget.AprilTag>(index), VisionPoseSupplier {
        override val visionTargets: List<VisionTarget.AprilTag>
            by logInputs.valueList(default = VisionTarget.AprilTag.Dummy){
                val data = latestResult
                if (!data.hasTargets() || isSimulation() || pipelineIndex != index){
                    return@valueList listOf()
                }else if (pipelineIndex != index){
                    println("AprilTag photon camera pipeline index is incorrect!")
                    return@valueList listOf()
                }

                return@valueList data
                    .getTargets()
                    .map{ toAprilTagTarget(it, data.timestampSeconds.ofUnit(seconds)) }
            }

        val poseEstimator = PhotonPoseEstimator(
            ChargerRobot.APRILTAG_LAYOUT,
            poseEstimationStrategy,
            this@ChargerPhotonCamera,
            robotToCamera.inUnit(meters)
        ).apply{
            setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
        }

        override val cameraYaw: Angle
            get() = Angle(robotToCamera.rotation.z)

        override val robotPoseEstimates: List<Measurement<UnitPose2d>>
            by logInputs.valueList(default = Measurement(UnitPose2d(), Time(0.0))){
                if (isSimulation() || pipelineIndex != index || disablePoseEstimation) {
                    listOf()
                }else{
                    when(val signal = poseEstimator.update()){
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
    }


    public inner class ObjectPipeline(
        index: Int,
        /**
         * The namespace of which the Photon camera Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): PhotonCameraPipeline<VisionTarget.Object>(index) {
        override val visionTargets: List<VisionTarget.Object>
            by logInputs.valueList(default = VisionTarget.Object.Dummy){
                val data = latestResult
                if (!data.hasTargets() || isSimulation()){
                    return@valueList listOf()
                }else if (pipelineIndex != index){
                    println("Object photon camera pipeline index is incorrect!")
                    return@valueList listOf()
                }

                return@valueList data
                    .getTargets()
                    .map{ toObjectTarget(it, data.timestampSeconds.ofUnit(seconds)) }
            }
    }



    abstract inner class PhotonCameraPipeline<T: VisionTarget>(val index: Int): VisionPipeline<T>{
        init{
            ensureIndexValid(index)
            // only reset pipeline if there is currently 1 pipeline index or less.
            if (allIndexes.size < 1){
                reset()
            }
        }

        final override fun reset(){
            // property access syntax(was getPipelineIndex() and setPipelineIndex())
            if (pipelineIndex != index){
                pipelineIndex = index
                println("Photon Camera with name $name has had it's pipeline reset to $index")
            }else{
                println("Photon Camera with name $name is on a pipeline with index $index")
            }
        }

        override val cameraConstants = VisionCameraConstants(
            "Photon Camera " + this@ChargerPhotonCamera.name,
            this@ChargerPhotonCamera.robotToCamera
        )
    }
}