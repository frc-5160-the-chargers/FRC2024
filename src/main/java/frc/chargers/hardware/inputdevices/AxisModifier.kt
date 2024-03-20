package frc.chargers.hardware.inputdevices

typealias TriggerValue = Double

/**
 * Represents a generic function that can modify an input axis.
 *
 * A functional interface is used to remove runtime overhead from boxed Doubles.
 */
fun interface AxisModifier{
    fun modifyAxis(input: TriggerValue): TriggerValue
}