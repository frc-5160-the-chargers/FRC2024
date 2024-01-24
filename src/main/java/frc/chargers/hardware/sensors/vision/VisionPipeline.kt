@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import org.photonvision.PhotonUtils
import kotlin.math.pow
import kotlin.math.sqrt

public typealias AprilTagVisionPipeline = VisionPipeline<VisionTarget.AprilTag>

public typealias ObjectVisionPipeline = VisionPipeline<VisionTarget.Object>

/**
 * Represents a generic vision system that can detect one type of target and compile data about it.
 *
 * In a multi-pipeline vision system, this interface should represent 1 pipeline.
 */
public interface VisionPipeline<R: VisionTarget> {

    /**
     * Fetches the full vision data of the [VisionPipeline]; These are all from the exact same timestamp.
     * A null value represents the vision data being invalid/the pipeline having no available targets.
     *
     * This is intended to be used as a getter variable.
     */
    public val visionData: NonLoggableVisionData<R>?

    /**
     * Camera constants specific to the overarching vision camera.
     */
    public val cameraConstants: VisionCameraConstants

    /**
     * Requires the overarching vision camera of the pipeline,
     * and performs any proper initialization needed; depending on the camera type.
     * This method should ```always``` be called before you use the vision pipeline.
     *
     * Initialization can include: resetting the pipeline index/mode, performing crosshair offsets, etc.
     */
    public fun requireAndReset()

    /**
     * Removes the requirement of the overarching vision camera
     * of the pipeline.
     */
    public fun removeRequirement()





    /**
     * Fetches the current best target of the [VisionPipeline].
     *
     * The values fetched here are not nessecarily from the exact same timestamp.
     */
    public val bestTarget: R?
        get() = visionData?.bestTarget

    /**
     * Calculates the horizontal distance to a target, utilizing the pitch and height of the target.
     */
    public fun distanceToTarget(
        targetHeight: Distance,
        target: R? = bestTarget
    ): Distance? = if (target != null){
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraConstants.lensHeight.inUnit(meters),
            targetHeight.inUnit(meters),
            cameraConstants.mountAngle.inUnit(radians),
            target.ty.ofUnit(degrees).inUnit(radians)
        ).ofUnit(meters)
    }else{
        null
    }

    /**
     * Calculates the diagonal distance to the target.
     */
    public fun diagonalDistanceToTarget(targetHeight: Distance, target: R? = bestTarget): Distance? {
        val horizontalDistance = distanceToTarget(targetHeight, target) ?: return null
        return Distance(
            sqrt(horizontalDistance.siValue.pow(2) + targetHeight.siValue.pow(2.0))
        )
    }

}

