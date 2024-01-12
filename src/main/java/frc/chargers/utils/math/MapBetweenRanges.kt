@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity

public fun Double.mapBetweenRanges(from: ClosedRange<Double>, to: ClosedRange<Double>): Double {
    require(this in from) { "An error has occured: your value($this) is not within the starting range($from)." }

    val proportionIntoRange: Double = (this - from.start) / (from.endInclusive - from.start)
    val distanceIntoToRange: Double = proportionIntoRange * (to.endInclusive - to.start)
    return to.start + distanceIntoToRange
}


public fun <D: Dimension<*,*,*,*>> Quantity<D>.mapBetweenRanges(
    from: ClosedRange<Quantity<D>>,
    to: ClosedRange<Quantity<D>>
): Quantity<D> = Quantity(
    this.siValue.mapBetweenRanges(
        from.start.siValue..from.endInclusive.siValue,
        to.start.siValue..to.endInclusive.siValue
    )
)