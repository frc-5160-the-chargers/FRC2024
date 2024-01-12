@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu.gyroscopes

/**
 * A [HeadingProvider] that can be zeroed.
 */
public interface ZeroableHeadingProvider: HeadingProvider {
    /**
     * Zeroes the heading of the [HeadingProvider].
     */
    public fun zeroHeading()

}