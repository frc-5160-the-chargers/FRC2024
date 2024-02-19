package frc.chargers.hardware.sensors.vision

import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d

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
     * The x, y, z and rotational components of the camera,
     * with respect to the origin of the robot.
     */
    val robotToCameraTransform: UnitTransform3d
)