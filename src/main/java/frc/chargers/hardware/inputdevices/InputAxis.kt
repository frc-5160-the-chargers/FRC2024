@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Frequency
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.utils.math.preserveSign

public typealias TriggerValue = Double

/**
 * An extension of a Double-supplying function designed for pre-processing driver controller axis inputs.
 * Credits go to FR 6995 for inspiring this class!
 *
 * Implements the () -> Double interface, so it is callable as a function.
 */
public class InputAxis(
    private val get: () -> TriggerValue
): () -> TriggerValue {
    private val modifiers: MutableList<AxisModifier> = mutableListOf()

    public fun withModifier(modifier: AxisModifier): InputAxis{
        this.modifiers.add(modifier)
        return this
    }

    public fun withModifiers(vararg modifiers: AxisModifier): InputAxis{
        this.modifiers.addAll(modifiers)
        return this
    }

    public fun applyDeadband(value: Double): InputAxis =
        withModifier { MathUtil.applyDeadband(it, value, 1.0) }

    public fun applyMultiplier(value: Double): InputAxis =
        withModifier { it * value }

    public fun applyEquation(equation: Polynomial): InputAxis =
        withModifier(equation)

    public fun mapToRange(range: ClosedRange<Double>): InputAxis =
        withModifier{ it.mapBetweenRanges(0.0..1.0, range) }


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


    public fun getBaseValue(): TriggerValue = get()

    override fun invoke(): TriggerValue {
        var value = get()
        modifiers.forEach{
            value = it.modifyAxis(value)
        }
        return value
    }
}

/**
 * Represents a generic function that can modify an input axis.
 */
public fun interface AxisModifier{
    public fun modifyAxis(input: TriggerValue): TriggerValue
}
