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
public interface VisionPipeline<T: VisionTarget> {
    /**
     * Fetches the full vision data of the [VisionPipeline]. This list includes all valid vision targets
     *
     * This is intended to be used as a getter variable.
     */
    public val visionTargets: List<T>

    /**
     * Camera constants specific to the overarching vision camera.
     */
    public val cameraConstants: VisionCameraConstants

    /**
     * Resets the overarching vision camera; configuring the correct settings for this pipeline to return proper results.
     *
     * This often includes changing the pipeline index of the camera to the appropriate value.
     */
    public fun reset()





    /**
     * Fetches the current best target of the [VisionPipeline].
     */
    public val bestTarget: T?
        get(){
            // fetches vision data once
            val data = visionTargets

            return if (data.isEmpty()){
                null
            }else{
                visionTargets[0]
            }
        }

    /**
     * Calculates the horizontal distance to a target, utilizing the pitch and height of the target.
     */
    public fun distanceToTarget(
        targetHeight: Distance,
        target: T? = bestTarget
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
    public fun diagonalDistanceToTarget(targetHeight: Distance, target: T? = bestTarget): Distance? {
        val horizontalDistance = distanceToTarget(targetHeight, target) ?: return null
        return Distance(
            sqrt(horizontalDistance.siValue.pow(2) + targetHeight.siValue.pow(2.0))
        )
    }
}

