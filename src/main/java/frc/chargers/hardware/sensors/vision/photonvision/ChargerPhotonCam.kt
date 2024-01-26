@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision.photonvision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj.RobotBase.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.vision.*
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*

/**
 * A wrapper over PhotonVision's [PhotonCamera], with support for AdvantageKit and more integration within ChargerLib.
 */
public class ChargerPhotonCam(
    name: String,
    public val lensHeight: Distance,
    public val mountAngle: Angle
): PhotonCamera(name) {
    private var required: Boolean = false

    private val allIndexes: MutableList<Int> = mutableListOf()

    private fun ensureIndexValid(index: Int){
        require (index !in allIndexes){ "There is already a PhotonVision pipeline with index $index." }
        allIndexes.add(index)
    }


    init{
        ChargerRobot.runPeriodically{
            Logger.recordOutput("PhotonCamera$name/pipelineIndex", pipelineIndex)
        }
    }

    /**
     * Represents a photonvision pipeline that can detect AprilTags.
     */
    public inner class AprilTagPipeline(
        index: Int,
        /**
         * The namespace of which the Photon Camera Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): PhotonCameraPipeline<VisionTarget.AprilTag>(index) {

        override val visionData: List<VisionTarget.AprilTag>
            by logInputs.valueList(default = VisionTarget.AprilTag.Dummy){
                val data = latestResult
                if (!data.hasTargets() || isSimulation() || pipelineIndex != index){
                    return@valueList listOf()
                }

                val bestTarget = data.bestTarget
                val otherTargets = data.getTargets()
                otherTargets.remove(bestTarget)

                return@valueList listOf(
                    toAprilTagTarget(bestTarget, data.timestampSeconds.ofUnit(seconds)),
                    *otherTargets.map{toAprilTagTarget(it, data.timestampSeconds.ofUnit(seconds))}.toTypedArray()
                )
            }
    }


    /**
     * Represents the pose estimation component of an apriltag pipeline.
     */
    public inner class AprilTagPoseEstimator(
        /**
         * The pipeline index where apriltag pose estimation takes place.
         * This should be equivalent to the pipeline index of the corresponding [AprilTagPipeline].
         */
        aprilTagPipelineIndex: Int,
        robotToCamera: UnitTransform3d,
        logInputs: LoggableInputsProvider,

        fieldTags: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
        strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    ): PhotonPoseEstimator(
        fieldTags,
        strategy,
        this@ChargerPhotonCam,
        robotToCamera.inUnit(meters)
    ), VisionPoseSupplier {
        override val cameraYaw: Angle
            get() = Angle(robotToCameraTransform.rotation.z)

        init{
            setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        }

        override val robotPoseEstimates: List<Measurement<UnitPose2d>>
            by logInputs.valueList(default = Measurement(UnitPose2d(), Time(0.0))){
                if (isSimulation() || pipelineIndex != aprilTagPipelineIndex) {
                    listOf()
                }else{
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

        override val visionData: List<VisionTarget.Object>
            by logInputs.valueList(default = VisionTarget.Object.Dummy){
                val data = latestResult
                if (!data.hasTargets() || isSimulation() || pipelineIndex != index){
                    return@valueList listOf()
                }

                val bestTarget = data.bestTarget
                val otherTargets = data.getTargets()
                otherTargets.remove(bestTarget)

                return@valueList listOf(
                    toObjectTarget(bestTarget, data.timestampSeconds.ofUnit(seconds)),
                    *otherTargets.map{ toObjectTarget(it, data.timestampSeconds.ofUnit(seconds)) }.toTypedArray()
                )
            }
    }



    abstract inner class PhotonCameraPipeline<T: VisionTarget>(val index: Int): VisionPipeline<T>{
        init{
            ensureIndexValid(index)
            // only reset pipeline if there is currently 1 pipeline index or less.
            if (allIndexes.size <= 1){
                resetPipeline()
            }
        }

        private fun resetPipeline(){
            // property access syntax(was getPipelineIndex() and setPipelineIndex())
            if (pipelineIndex != index){
                pipelineIndex = index
                println("Photon Camera with name $name has had it's pipeline reset to $index")
            }else{
                println("Photon Camera with name $name is on a pipeline with index $index")
            }
        }

        override fun requireAndReset(){
            if (required){
                error("A Photon Camera with name '$name' has been required in 2 different places. \n " +
                        "Make sure to call pipeline.isRequired = false at the end of all commands!"
                )
            }
            required = true
            resetPipeline()
        }

        override fun removeRequirement(){
            if (!required){
                println("A requirement was removed from a PhotonCamera; however, this requirement was never set in the first place.")
            }
            required = false
        }

        override val cameraConstants = VisionCameraConstants(
            "Photon Camera " + this@ChargerPhotonCam.name,
            this@ChargerPhotonCam.lensHeight,
            this@ChargerPhotonCam.mountAngle
        )
    }

    private fun toAprilTagTarget(target: PhotonTrackedTarget, timestamp: Time) =
        VisionTarget.AprilTag(
            timestamp,
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            fiducialId = target.fiducialId,
            targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
        )

    private fun toObjectTarget(target: PhotonTrackedTarget, timestamp: Time) =
        VisionTarget.Object(
            timestamp,
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            classId = null // photonvision does not support class id's so far.
        )
}