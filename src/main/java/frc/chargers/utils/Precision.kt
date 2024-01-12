@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity

/**
 * A sealed class enum that holds data about the Precision of a certain mechanism or class.
 *
 * Example usage:
 * ```
 * val precision: Precision<AngleDimension> = Precision.Within(5.0.degrees)
 *
 * when(precision){
 *      Precision.AllowOvershoot -> println("Precision is allow overshoot")
 *
 *      // no need for an else branch; sealed classes know all of their inheritors
 *      // at runtime; similar to rust enums
 *      // allowableError is only a property of Precision.Within; this enforces compile time safety
 *      is Precision.Within -> println("Allowable Error: " + precision.allowableError)
 * }
 */
public sealed class Precision<out D : Dimension<*,*,*,*>> {
    /**
     * Represents a [Precision] that allows overshoot.
     */
    public data object AllowOvershoot : Precision<Nothing>()

    /**
     * Represents a [Precision] that allows for a certain margin of error.
     */
    public class Within<D : Dimension<*,*,*,*>>(public val allowableError: ClosedRange<Quantity<D>>) : Precision<D>() {
        public constructor(margin: Quantity<D>) : this(-margin..margin)
    }
}

/**
 * Determines whether a quantity is within a specified [Precision].
 *
 * If the precision is a [Precision.Within], it will return true if the value is between the allowable error or not.
 * Otherwise, it will return false.
 */
public fun <D: Dimension<*,*,*,*>> Quantity<D>.within(precision: Precision<D>): Boolean =
    if (precision is Precision.Within){
        this in precision.allowableError
    }else{
        false
    }










