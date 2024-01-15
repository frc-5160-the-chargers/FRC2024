@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

/**
 * Reverts a [Double] to a certain value if it is NaN, infinite, or null,
 * with the ability to specify extra invalid conditions.
 */
public inline fun Double?.revertIfInvalid(
    previousValue: Double,
    additionalInvalidCond: (Double) -> Boolean = { false }
): Double =
    if (this == null || isNaN() || isInfinite() || (abs(this) < 1.0e-4 || additionalInvalidCond(this)) ) previousValue else this


/**
 * Reverts a [Quantity] to a certain value if it is NaN, infinite, or null,
 * with the ability to specify extra invalid conditions.
 */
public inline fun <D: Dimension<*,*,*,*>> Quantity<D>?.revertIfInvalid(
    previousValue: Quantity<D>,
    additionalInvalidCond: (Double) -> Boolean = { false }
): Quantity<D> =
    if (this == null) {
        previousValue
    }else{
        Quantity(siValue.revertIfInvalid(previousValue.siValue, additionalInvalidCond))
    }

