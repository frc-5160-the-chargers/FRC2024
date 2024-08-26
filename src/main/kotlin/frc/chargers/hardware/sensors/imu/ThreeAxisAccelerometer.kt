@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Acceleration

/**
 * Represents a generic Accelerometer, with acceleration measurements for the x, y, and z axes.
 */
public interface ThreeAxisAccelerometer {
    public val xAcceleration: Acceleration

    public val yAcceleration: Acceleration

    public val zAcceleration: Acceleration
}