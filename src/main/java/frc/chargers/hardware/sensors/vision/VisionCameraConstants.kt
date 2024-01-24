package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance

/**
 * A utility class that stores constants related a single vision camera module.
 */
data class VisionCameraConstants(
    val name: String,
    /**
     * How high the vision camera's lens is from the ground.
     */
    val lensHeight: Distance,
    /**
     * How many degrees the vision camera is from perfectly vertical.
     */
    val mountAngle: Angle
)