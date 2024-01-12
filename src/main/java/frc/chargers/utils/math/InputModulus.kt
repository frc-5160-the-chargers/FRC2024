@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.MathUtil

/**
 * Returns the modulus of a Double.
 *
 * @see MathUtil.inputModulus
 */
public fun Double.inputModulus(min: Double, max: Double): Double =
    MathUtil.inputModulus(this,min,max)

/**
 * Returns the modulus of a Double.
 *
 * @see MathUtil.inputModulus
 */
public fun Double.inputModulus(range: ClosedRange<Double>): Double =
    inputModulus(range.start,range.endInclusive)

/**
 * Returns the modulus of a [Quantity].
 *
 * @see MathUtil.inputModulus
 */
public fun <D: Dimension<*, *, *, *>> Quantity<D>.inputModulus(min: Quantity<D>, max: Quantity<D>): Quantity<D> =
    Quantity(siValue.inputModulus(min.siValue,max.siValue))

/**
 * Returns the modulus of a [Quantity].
 *
 * @see MathUtil.inputModulus
 */
public fun <D: Dimension<*, *, *, *>> Quantity<D>.inputModulus(range: ClosedRange<Quantity<D>>): Quantity<D> =
    inputModulus(range.start,range.endInclusive)
