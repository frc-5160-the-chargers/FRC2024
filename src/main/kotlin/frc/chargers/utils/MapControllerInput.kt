package frc.chargers.utils

import kotlin.math.abs

/**
 * Maps a value from a controller axis value(ranging from -1.0 to 1.0)
 * to a new [ClosedRange].
 *
 * For instance, if the target range is 1 to 7, and the controller input value
 * is 0.5, then the value will become 4. A -0.5 value would yield -4.
 */
fun Double.mapControllerInput(to: ClosedRange<Double>): Double {
    val sign = if (this > 0) 1 else -1
    val rangeLength = to.endInclusive - to.start
    return (abs(this) * rangeLength + to.start) * sign
}