@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.HeadingProvider

/**
 * Creates a [PositionEncoder] with a certain zero offset.
 */
public fun PositionEncoder.withOffset(zeroOffset: Angle): PositionEncoder = object: PositionEncoder {
    override val angularPosition: Angle
        get() = this@withOffset.angularPosition - zeroOffset
}

/**
 * Creates a [Encoder] with a certain zero offset.
 */
public fun Encoder.withOffset(zeroOffset: Angle): Encoder = object: Encoder by this{
    override val angularPosition: Angle
        get() = this@withOffset.angularPosition - zeroOffset
}

/**
 * Creates a [HeadingProvider] with a certain zero offset.
 */
public fun HeadingProvider.withOffset(zeroOffset: Angle): HeadingProvider = HeadingProvider{heading - zeroOffset}

/**
 * Creates a [HeadingProvider] that zeroes itself according to it's existing position.
 */
public fun HeadingProvider.withZero(): HeadingProvider = withOffset(heading)

