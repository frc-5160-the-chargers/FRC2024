@file:Suppress("unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Acceleration

/**
 * Represents a generic Accelerometer, with acceleration measurements for the x, y, and z axes.
 */
interface ThreeAxisAccelerometer {
    val xAcceleration: Acceleration

    val yAcceleration: Acceleration

    val zAcceleration: Acceleration
}