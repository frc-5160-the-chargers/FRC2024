@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import org.photonvision.PhotonUtils

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
    public fun robotToTargetDistance(
        targetHeight: Distance,
        target: T? = bestTarget
    ): Distance? = if (target != null){
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraConstants.robotToCameraTransform.z.inUnit(meters),
            targetHeight.inUnit(meters),
            -cameraConstants.robotToCameraTransform.rotation.y,
            target.ty.ofUnit(degrees).inUnit(radians)
        ).ofUnit(meters) //+ cameraConstants.robotToCameraTransform.x
    }else{
        null
    }
}

