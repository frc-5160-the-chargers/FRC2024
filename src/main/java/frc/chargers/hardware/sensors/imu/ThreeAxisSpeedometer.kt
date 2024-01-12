@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Velocity

/**
 * Represents a Speedometer that can measure velocity in the x, y and z directions.
 */
public interface ThreeAxisSpeedometer {
    public val xVelocity: Velocity
    public val yVelocity: Velocity
    public val zVelocity: Velocity

    public companion object{
        /**
         * Inline syntax to create a generic [ThreeAxisSpeedometer].
         */
        public inline operator fun invoke(
            crossinline getXSpeed: () -> Velocity,
            crossinline getYSpeed: () -> Velocity,
            crossinline getZSpeed: () -> Velocity
        ): ThreeAxisSpeedometer = object: ThreeAxisSpeedometer {
            override val xVelocity: Velocity get() = getXSpeed()
            override val yVelocity: Velocity get() = getYSpeed()
            override val zVelocity: Velocity get() = getZSpeed()
        }
    }
}