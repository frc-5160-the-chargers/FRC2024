@file:Suppress("unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Angle

/**
 * A [HeadingProvider] that can be zeroed.
 */
interface ZeroableHeadingProvider: HeadingProvider {
    /**
     * Zeroes the heading of the [HeadingProvider].
     */
    fun zeroHeading(angle: Angle = Angle(0.0))
}