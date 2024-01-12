@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.imu.gyroscopes

import com.batterystaple.kmeasure.quantities.Angle


/**
 * Represents a generic gyroscope, with measurements for yaw, pitch, and roll.
 */
public interface ThreeAxisGyroscope: HeadingProvider {
    public val yaw: Angle

    public val pitch: Angle

    public val roll: Angle

    override val heading: Angle get() = yaw

    public companion object{
        /**
         * Inline syntax to create a generic [ThreeAxisGyroscope].
         */
        public inline operator fun invoke(
            crossinline getYaw: () -> Angle,
            crossinline getPitch: () -> Angle,
            crossinline getRoll: () -> Angle
        ): ThreeAxisGyroscope = object: ThreeAxisGyroscope{
            override val yaw: Angle get() = getYaw()
            override val pitch: Angle get() = getPitch()
            override val roll: Angle get() = getRoll()
        }
    }
}