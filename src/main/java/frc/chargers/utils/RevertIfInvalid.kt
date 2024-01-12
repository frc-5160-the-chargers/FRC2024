@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

/**
 * Reverts a [Double] to a certain value if it is NaN or infinite,
 * with the ability to specify extra invalid conditions.
 */
public fun Double.revertIfInvalid(
    previousValue: Double,
    additionalInvalidCond: Boolean = false
): Double =
    if (isNaN() || isInfinite() || (abs(this) < 1.0e-4) || additionalInvalidCond) previousValue else this


/**
 * Reverts a [Quantity] to a certain value if it is NaN or infinite,
 * with the ability to specify extra invalid conditions.
 */
public fun <D: Dimension<*,*,*,*>> Quantity<D>.revertIfInvalid(
    previousValue: Quantity<D>,
    additionalInvalidCond: Boolean = false
): Quantity<D> =
    Quantity(siValue.revertIfInvalid(previousValue.siValue, additionalInvalidCond))