@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity

/**
 * Represents a generic encoder.
 *
 * An encoder is a device affixed to a motor that measures its rotation.
 * Objects implementing [Encoder] interface support both position and velocity access;
 * use [PositionEncoder] if you only need position values.
 */
public interface Encoder : PositionEncoder {
    public val angularVelocity: AngularVelocity
}

/**
 * Represents an encoder that can measure the current angular position of a motor.
 */
public interface PositionEncoder {
    public val angularPosition: Angle
}