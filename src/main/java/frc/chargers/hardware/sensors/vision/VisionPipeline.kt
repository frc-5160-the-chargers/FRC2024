@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import org.photonvision.PhotonUtils
import kotlin.math.pow
import kotlin.math.sqrt


public typealias AprilTagVisionPipeline = VisionPipeline<VisionTarget.AprilTag>

public typealias MLVisionPipeline = VisionPipeline<VisionTarget.ML>

public typealias GenericVisionPipeline = VisionPipeline<VisionTarget.Generic>

/**
 * Represents a generic vision system that can detect one type of target and compile data about it.
 *
 * In a multi-pipeline vision system, this interface should represent 1 pipeline.
 */
public interface VisionPipeline<R: VisionTarget> {

    /**
     * Fetches the full vision data of the [VisionPipeline]; These are all from the exact same timestamp.
     * A null value represents the vision data being invalid/the pipeline having no available targets.
     */
    public val visionData: NonLoggableVisionData<R>?

    /**
     * How high the vision camera's lens is from the ground.
     */
    public val lensHeight: Distance

    /**
     * The mount angle describes how many degrees the vision camera is from perfectly vertical.
     */
    public val mountAngle: Angle

    /**
     * Resets the camera that the [VisionPipeline] belongs to in order to return proper results.
     * This can include setting the overall camera's pipeline index to the index specified.
     */
    public fun reset()

    /**
     * Requires the overarching vision camera
     * of the pipeline.
     */
    public fun require()

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
            lensHeight.inUnit(meters),
            targetHeight.inUnit(meters),
            mountAngle.inUnit(radians),
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

