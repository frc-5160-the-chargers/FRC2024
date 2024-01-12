@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.controls

/**
 * Represents a generic controller, which can produce a certain output when called.
 */
public fun interface Controller<out T> {
    public fun calculateOutput(): T
}