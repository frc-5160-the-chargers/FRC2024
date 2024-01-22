@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*

/**
 * Stores a produced Setpoint; with the appropriate setpoint value,
 * and additional feedforward control output.
 */
public data class Setpoint<S: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>>(
    /**
     * The value of the setpoint.
     */
    val value: Quantity<S>,
    /**
     * Additional control output provided by a [SetpointSupplier];
     * usually calculated via a feedforward.
     */
    val feedforwardOutput: Quantity<O>
)

/**
 * A class that can supply a certain setpoint to a [FeedbackController] or [Controller].
 */
public fun interface SetpointSupplier<S: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>>{

    /**
     * Fetches a calculated [Setpoint]
     * with the appropriate target goal.
     */
    public fun calculateSetpoint(target: Quantity<S>): Setpoint<S, O>

    /**
     * Fetches a calculated [Setpoint]
     * with the appropriate target goal;
     * with continuous input support.
     */
    public fun calculateSetpoint(
        target: Quantity<S>,
        continuousInputRange: ClosedRange<Quantity<S>>,
        measurement: Quantity<S>
    ): Setpoint<S, O> = calculateSetpoint(target)


    /**
     * A default setpoint supplier, with an optional feedforward
     * that is directly dependent on the target of the controller.
     *
     * For instance, in a velocity controller,
     * you would use this class and pass in the appropriate feedforward.
     */
    public class Default<I: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>>(
        private val ffEquation: (Quantity<I>) -> Quantity<O> = { Quantity(0.0) }
    ): SetpointSupplier<I,O>{
        override fun calculateSetpoint(target: Quantity<I>): Setpoint<I,O> =
            Setpoint(target,ffEquation(target))
    }

}