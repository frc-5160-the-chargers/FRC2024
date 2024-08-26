@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

/**
 * Maps a [Double] between 2 ranges.
 * ```
 * val newValue = 0.5.mapBetweenRanges(from = 0.0..1.0, to = 0.0..2.0)
 * println(newValue) // will print 1.0; 0.5 is halfway between 0.0 and 1.0, (2.0 - 0.0) / 2 = 1.0
 */
public fun Double.mapBetweenRanges(from: ClosedRange<Double>, to: ClosedRange<Double>): Double {
    require(this in from) { "An error has occurred: your value($this) is not within the starting range($from)." }

    val proportionIntoRange: Double = (this - from.start) / (from.endInclusive - from.start)
    val distanceIntoToRange: Double = proportionIntoRange * (to.endInclusive - to.start)
    return to.start + distanceIntoToRange
}

/**
 * Maps a value from a controller axis to a new [ClosedRange].
 *
 * @see mapBetweenRanges
 */
public fun Double.mapAxisTo(to: ClosedRange<Double>): Double {
    val sign = if (this > 0) -1 else 1
    return abs(this).mapBetweenRanges(from = 0.0..1.0, to = to) * sign
}

/**
 * Maps a [Quantity] between 2 ranges.
 * @see mapBetweenRanges
 */
public fun <D: Dimension<*,*,*,*>> Quantity<D>.mapBetweenRanges(
    from: ClosedRange<Quantity<D>>,
    to: ClosedRange<Quantity<D>>
): Quantity<D> = Quantity(
    this.siValue.mapBetweenRanges(
        from.start.siValue..from.endInclusive.siValue,
        to.start.siValue..to.endInclusive.siValue
    )
)