@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*

/**
 * Represents a generic encoder.
 *
 * An encoder is a device affixed to a motor that measures its rotation.
 * Objects implementing [Encoder] interface support both position and velocity access;
 * use either [PositionEncoder] or [VelocityEncoder] if only one is required.
 */
public interface Encoder : PositionEncoder, VelocityEncoder

/**
 * Represents an encoder that can measure the current angular position of a motor.
 */
public interface PositionEncoder {
    public val angularPosition: Angle
}

/**
 * Represents an encoder that can measure the current angular velocity of a motor.
 */
public interface VelocityEncoder {
    public val angularVelocity: AngularVelocity
}