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

    /**
     * Adds the [offset] to the [angularPosition] of the encoder.
     */
    override operator fun plus(offset: Angle): Encoder =
        object: Encoder by this { override val angularPosition: Angle get() = this@Encoder.angularPosition + offset }

    /**
     * Subtracts the [offset] from the [angularPosition] of the encoder.
     */
    override operator fun minus(offset: Angle): Encoder = this + (-offset)
}

/**
 * Represents an encoder that can measure the current angular position of a motor.
 */
public interface PositionEncoder {
    public val angularPosition: Angle

    /**
     * Adds the [offset] to the [angularPosition] of the encoder.
     */
    public operator fun plus(offset: Angle): PositionEncoder =
        object: PositionEncoder { override val angularPosition: Angle get() = this@PositionEncoder.angularPosition + offset }

    /**
     * Subtracts the [offset] from the [angularPosition] of the encoder.
     */
    public operator fun minus(offset: Angle): PositionEncoder = this + (-offset)
}
