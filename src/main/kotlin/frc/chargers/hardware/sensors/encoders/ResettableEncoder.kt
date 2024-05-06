@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle

/**
 * An encoder that allows resetting zero values.
 * Particularly useful for absolute encoders or for relative encoders after homing.
 *
 * Depending on the underlying implementation, new zeroes
 * may or may not persist between restarts or runs of the program.
 */
public interface ResettableEncoder : Encoder {
    /**
     * Sets the specified angular position to be the new zero.
     */
    public fun setZero(newZero: Angle)

    public fun setZero() { setZero(angularPosition) }
}

