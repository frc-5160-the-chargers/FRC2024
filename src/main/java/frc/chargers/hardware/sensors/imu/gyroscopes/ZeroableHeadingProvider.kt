@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu.gyroscopes

import com.batterystaple.kmeasure.quantities.Angle

/**
 * A [HeadingProvider] that can be zeroed.
 */
public interface ZeroableHeadingProvider: HeadingProvider {
    /**
     * Zeroes the heading of the [HeadingProvider].
     */
    public fun zeroHeading(angle: Angle = Angle(0.0))

}