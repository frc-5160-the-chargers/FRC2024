@file:Suppress("unused")
package frc.chargers.utils.math

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

private var kEpsilon = 1E-9

fun configureEpsilon(value: Double){
    kEpsilon = value
}

infix fun Double.epsilonEquals(other: Double): Boolean =
    abs(this - other) < kEpsilon

infix fun <D: Dimension<*,*,*,*>> Quantity<D>.epsilonEquals(other: Quantity<D>): Boolean =
    siValue epsilonEquals other.siValue