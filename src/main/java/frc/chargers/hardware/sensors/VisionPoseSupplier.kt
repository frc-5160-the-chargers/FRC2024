@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.*
import frc.chargers.utils.BasicMeasurement
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

/**
 * Represents a vision system that can supply a vision pose estimate.
 *
 * This interface uses the [BasicMeasurement] / Measurement classes
 * to hold the pose, as well as the timestamp; with null representing an invalid(or no)
 * vision measurement.
 */
public interface VisionPoseSupplier{
    public val robotPoseEstimate: BasicMeasurement<UnitPose2d>?

    public val cameraYaw: Angle
}


