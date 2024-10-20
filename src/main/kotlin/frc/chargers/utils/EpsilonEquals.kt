@file:Suppress("unused")
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

private var kEpsilon = 1E-9

fun configureEpsilon(value: Double){
    kEpsilon = value
}

infix fun Double.epsilonEquals(other: Double): Boolean =
    abs(this - other) < kEpsilon

infix fun <D: AnyDimension> Quantity<D>.epsilonEquals(other: Quantity<D>): Boolean =
    siValue epsilonEquals other.siValue