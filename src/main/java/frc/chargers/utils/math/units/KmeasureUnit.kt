@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity

/**
 * In kmeasure, Units are considered the same as quantities;
 * for example, the unit radians is actually a value, storing a Quantity(1.0).
 *
 * This "quantity" representation is meant to be considered a conversion factor,
 * to convert in and out of the unit itself.
 *
 * this typealias simply serves to increase code clarity within the library.
 */
public typealias KmeasureUnit<D> = Quantity<D>


/**
 * Gets an SI unit representation of a specific quantity and/or dimension.
 */
public fun <D: Dimension<*,*,*,*>> siUnit(): KmeasureUnit<D> = KmeasureUnit(1.0)