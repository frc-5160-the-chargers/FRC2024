@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Frequency
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import frc.chargers.framework.Loggable
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.utils.math.preserveSign
import kotlin.math.abs
import kotlin.properties.ReadOnlyProperty


/**
 * An extension of a Double-supplying function designed for pre-processing driver controller axis inputs.
 * Credits go to FR 6995 for inspiring this class!
 *
 * Implements the () -> Double interface, so it is callable as a function.
 */
public class InputAxis(
    private val get: () -> TriggerValue
): () -> TriggerValue, ReadOnlyProperty<Any?, Double> by ReadOnlyProperty({ _, _ -> get() }), Loggable {

    override val namespace = ""

    public fun getBaseValue(): TriggerValue = get()

    override operator fun invoke(): TriggerValue {
        var value = get()
        for (modifier in modifiers){
            value = modifier.modifyAxis(value)
        }
        return value
    }

    private val modifiers: MutableList<AxisModifier> = mutableListOf()

    public fun withModifier(modifier: AxisModifier): InputAxis{
        this.modifiers.add(modifier)
        return this
    }

    public fun withModifiers(vararg modifiers: AxisModifier): InputAxis{
        this.modifiers.addAll(modifiers)
        return this
    }

    public fun withDeadband(value: Double): InputAxis =
        withModifier { MathUtil.applyDeadband(it, value, 1.0) }

    public fun withMultiplier(value: Double): InputAxis =
        withModifier { it * value }

    public fun withEquation(equation: Polynomial): InputAxis =
        withModifier(equation)

    public fun mapToRange(range: ClosedRange<Double>): InputAxis =
        withModifier{
            val initialValue = abs(it).mapBetweenRanges(from = 0.0..1.0, to = range)
            if (it < 0.0){
                -initialValue
            }else{
                initialValue
            }
        }

    public fun rateLimit(positiveLimit: Frequency, negativeLimit: Frequency): InputAxis{
        val rateLimiter = SlewRateLimiter(positiveLimit.siValue, negativeLimit.siValue, 0.0)
        return withModifier{ rateLimiter.calculate(it) }
    }

    public fun rateLimit(limit: Frequency): InputAxis{
        val rateLimiter = SlewRateLimiter(limit.siValue)
        return withModifier{ rateLimiter.calculate(it) }
    }

    public fun square(): InputAxis =
        withModifier{ (it * it).preserveSign(it) }

    public fun invert(): InputAxis =
        withModifier{ it * -1.0 }

    public fun invertWhen(booleanSupplier: () -> Boolean): InputAxis =
        withModifier{ it * if (booleanSupplier()) -1.0 else 1.0 }

    public fun log(namespace: String): InputAxis =
        withModifier{
            log(namespace, it)
            it
        }
}
