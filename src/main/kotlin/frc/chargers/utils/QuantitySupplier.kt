package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity

/**
 * A quantity supplying function with no boxing overhead.
 *
 * A regular () -> Quantity<D> supplier would be boxed due to Quantity<D> being used as a generic;
 * however, this is not the case here.
 *
 * Note: A Quantity<D> is a
 */
fun interface QuantitySupplier<D: Dimension<*, *, *, *>> {
    fun get(): Quantity<D>
}