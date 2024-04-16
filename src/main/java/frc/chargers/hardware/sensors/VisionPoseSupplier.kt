@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

/**
 * Represents a vision system that can supply a vision pose estimate.
 *
 * This interface uses the [BasicMeasurement] class
 * to hold the pose, as well as the timestamp;
 * however, most classes implementing this interface should use the
 * [frc.chargers.utils.Measurement] class instead, which allows for the data to be logged to an AdvantageKit field.
 */
public interface VisionPoseSupplier{
    public val robotPoseEstimates: List<Measurement<UnitPose2d>>

    public val cameraYaw: Angle
}


