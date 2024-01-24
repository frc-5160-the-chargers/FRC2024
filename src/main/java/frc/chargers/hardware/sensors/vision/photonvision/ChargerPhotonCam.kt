@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision.photonvision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj.RobotBase.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.sensors.vision.*
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*

/**
 * A wrapper over PhotonVision's [PhotonCamera], built for ChargerLib.
 */
public class ChargerPhotonCam(
    name: String,
    public val lensHeight: Distance,
    public val mountAngle: Angle
): PhotonCamera(name){
    private var required: Boolean = false

    private val allIndexes: MutableList<Int> = mutableListOf()

    private fun ensureIndexValid(index: Int){
        require (index !in allIndexes){ "There is already a PhotonVision pipeline with index $index." }
        allIndexes.add(index)
    }

    /**
     * Represents a
     */
    public inner class AprilTagPipeline(
        index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): PhotonCameraPipeline<VisionTarget.AprilTag>(index) {

        override val visionData: VisionData<VisionTarget.AprilTag>?
            by logInputs.nullableValue(default = emptyAprilTagVisionData()){
                val data = latestResult
                if (!data.hasTargets() || isSimulation()){
                    return@nullableValue null
                }

                val bestTarget = data.bestTarget
                val otherTargets = data.getTargets()
                otherTargets.remove(bestTarget)

                return@nullableValue VisionData(
                    data.timestampSeconds.ofUnit(seconds),
                    toAprilTagTarget(bestTarget),
                    otherTargets.map{toAprilTagTarget(it)}
                )
            }
    }


    public inner class AprilTagPoseEstimator(
        /**
         * The pipeline index where apriltag pose estimation takes place.
         * This should be equivalent to the pipeline index of the corresponding [AprilTagPipeline].
         */
        aprilTagPipelineIndex: Int,
        logInputs: LoggableInputsProvider,
        robotToCamera: UnitTransform3d,

        fieldTags: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
        strategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    ): PhotonPoseEstimator(
        fieldTags,
        strategy,
        this@ChargerPhotonCam,
        robotToCamera.inUnit(meters)
    ), VisionPoseSupplier{
        override val cameraYaw: Angle
            get() = Angle(robotToCameraTransform.rotation.z)

        init{
            setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        }

        override val robotPoseEstimate: Measurement<UnitPose2d>?
            by logInputs.nullableValue(
                default = Measurement(UnitPose2d(), fpgaTimestamp())
            ){
                if (isSimulation() || pipelineIndex != aprilTagPipelineIndex) {
                    null
                }else{
                    when(val signal = update()){
                        Optional.empty<EstimatedRobotPose>() -> null

                        else -> Measurement(
                            value = UnitPose2d(signal.get().estimatedPose.toPose2d()),
                            timestamp = signal.get().timestampSeconds.ofUnit(seconds)
                        )
                    }
                }
            }
    }


    public inner class ObjectPipeline(
        index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): PhotonCameraPipeline<VisionTarget.Object>(index) {

        override val visionData: VisionData<VisionTarget.Object>?
            by logInputs.nullableValue(default = emptyObjectVisionData()){
                val data = latestResult
                if (!data.hasTargets() || isSimulation()){
                    return@nullableValue null
                }

                val bestTarget = data.bestTarget
                val otherTargets = data.getTargets()
                otherTargets.remove(bestTarget)

                return@nullableValue VisionData(
                    data.timestampSeconds.ofUnit(seconds),
                    toObjectTarget(bestTarget),
                    otherTargets.map{ toObjectTarget(it) }
                )
            }
    }



    abstract inner class PhotonCameraPipeline<T: VisionTarget>(val index: Int): VisionPipeline<T>{
        init{
            ensureIndexValid(index)
            reset()
        }

        final override fun reset(){
            pipelineIndex = index
            println("Photon Camera with name $name has had it's pipeline reset to $index")
        }

        override fun require(){
            if (required){
                error("A Photon Camera with name '$name' has been required in 2 different places. \n " +
                        "Make sure to call pipeline.isRequired = false at the end of all commands!"
                )
            }
            required = true
        }

        override fun removeRequirement(){
            if (!required){
                println("A requirement was removed from a PhotonCamera; however, this requirement was never set in the first place.")
            }
            required = false
        }

        override val lensHeight: Distance = this@ChargerPhotonCam.lensHeight

        override val mountAngle: Angle = this@ChargerPhotonCam.mountAngle
    }

    private fun toAprilTagTarget(target: PhotonTrackedTarget) =
        VisionTarget.AprilTag(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            fiducialId = target.fiducialId,
            targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
        )

    private fun toObjectTarget(target: PhotonTrackedTarget) =
        VisionTarget.Object(
            tx = target.yaw,
            ty = target.pitch,
            areaPercent = target.area,
            classId = null // photonvision does not support class id's so far.
        )
}