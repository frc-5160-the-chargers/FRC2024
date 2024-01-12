@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math.equations

import kotlin.math.pow

/**
 * A class representing a polynomial function.
 *
 * Implements the (Double) -> (Double) interface, so can be called as a function.
 *
 * Coefficients are ordered from most to least significant;
 *
 * For example, `Polynomial(1.0, 2.0, 3.0)` represents the polynomial
 * x^2 + 2x + 3.
 */
public data class Polynomial(val coefficients: List<Double>) : (Double) -> Double {
    public constructor(vararg coefficients: Number) : this(coefficients.map{it.toDouble()})

    init{
        require(coefficients.isNotEmpty()){"Your polynomial must have at least 1 coefficient."}
    }

    /**
     * Makes the polynomial preserve it's sign:
     * when the input is negative, it will always return a negative value,
     * and if the input is positive, it will always return a positive value.
     */
    public fun preserveSign(): (Double) -> Double = {
        val result = invoke(it)
        result * if ( (result < 0 && it > 0) || (result > 0 && it < 0) ) -1 else 1
    }

    private val totalCoeffLength = coefficients.size

    override fun invoke(x: Double): Double =
        coefficients
            .asSequence()
            .mapIndexed { index, coeff ->
                coeff * x.pow(totalCoeffLength - index - 1)
            }
            .sum()
}