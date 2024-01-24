package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance

/**
 * A utility class that stores constants related a single vision camera.
 *
 * Different pipelines should have the same instance of this class.
 */
data class VisionCameraConstants(
    /**
     * The name of the overarching vision camera.
     * This should be the same for every pipeline that belongs to the same camera.
     */
    val cameraName: String,
    /**
     * How high the vision camera's lens is from the ground.
     */
    val lensHeight: Distance,
    /**
     * How many degrees the vision camera is from perfectly vertical.
     */
    val mountAngle: Angle
)