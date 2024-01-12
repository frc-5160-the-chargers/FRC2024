@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math.equations

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.math.abs

private var kEpsilon = 1E-9

public fun configureEpsilon(value: Double){
    kEpsilon = value
}

public infix fun Double.epsilonEquals(other: Double): Boolean =
    abs(this - other) < kEpsilon

public infix fun <D: Dimension<*,*,*,*>> Quantity<D>.epsilonEquals(other: Quantity<D>): Boolean =
    siValue epsilonEquals other.siValue