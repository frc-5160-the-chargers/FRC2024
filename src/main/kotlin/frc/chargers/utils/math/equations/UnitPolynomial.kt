@file:Suppress("unused")
package frc.chargers.utils.math.equations

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import kotlin.math.pow

/**
 * A class representing a polynomial function, with Units support.
 *
 * Example usage:
 * ```
 * // this would be equal to 5x^2 + 6.1x + 7.2
 * val pol = UnitPolynomial(degrees to volts, 5.1,6.1,7.2)
 * // you can also use a list
 * val pol2 = UnitPolynomial(degrees to volts, coefficientList)
 * ```
 * @see Polynomial
 */
data class UnitPolynomial<I: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>>(
    val units: Pair<Quantity<I>, Quantity<O>> = Quantity<I>(1.0) to Quantity(1.0),
    val coefficients: List<Double>
) : (Quantity<I>) -> Quantity<O> {
    constructor(
        unitsUsed: Pair<Quantity<I>, Quantity<O>> = Quantity<I>(1.0) to Quantity(1.0),
        vararg coefficients: Number
    ) : this(unitsUsed, coefficients.map{it.toDouble()})


    init{
        require(coefficients.isNotEmpty()){"Your polynomial must have at least 1 coefficient."}
    }

    /**
     * Makes the polynomial preserve it's sign:
     * when the input is negative, it will always return a negative value,
     * and if the input is positive, it will always return a positive value.
     */
    fun preserveSign(): (Quantity<I>) -> Quantity<O> = {
        val result = invoke(it)

        if ( (result.siValue < 0 && it.siValue > 0) || (result.siValue > 0 && it.siValue < 0) ){
            -result
        }else{
            result
        }
    }

    private val totalCoeffLength = coefficients.size

    override fun invoke(x: Quantity<I>): Quantity<O> =
        coefficients
        .asSequence()
        .mapIndexed { index, coeff ->
            coeff * x.inUnit(units.first).pow(totalCoeffLength - index - 1)
        }
        .sum()
        .ofUnit(units.second)
}

