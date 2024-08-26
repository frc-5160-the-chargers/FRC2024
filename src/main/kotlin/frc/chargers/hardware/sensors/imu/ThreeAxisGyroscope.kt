@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Angle


/**
 * Represents a generic gyroscope, with measurements for yaw, pitch, and roll.
 */
public interface ThreeAxisGyroscope: HeadingProvider {
    public val yaw: Angle

    public val pitch: Angle

    public val roll: Angle

    override val heading: Angle get() = yaw
}