@file:Suppress("unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Angle


/**
 * Represents a generic gyroscope, with measurements for yaw, pitch, and roll.
 */
interface ThreeAxisGyroscope: HeadingProvider {
    val yaw: Angle

    val pitch: Angle

    val roll: Angle

    override val heading: Angle get() = yaw
}