package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit

/**
 * A convenience function to convert a list of [Quantity]s to a [DoubleArray].
 * Useful for logging while incurring less overhead than list.map { it.inUnit(unit) }.toDoubleArray().
 */
fun <D: AnyDimension> List<Quantity<D>>.toDoubleArray(unit: Quantity<D>): DoubleArray {
    val doubleArray = DoubleArray(this.size)
    for (i in this.indices) {
        doubleArray[i] = this[i].inUnit(unit)
    }
    return doubleArray
}