package frc.chargers.utils

import kotlin.math.pow

/**
 * Squares a [Double] while preserving its positive/negative sign.
 */
fun Double.squareMagnitude(): Double {
    val sign = if (this < 0) -1.0 else 1.0
    val magnitude = this.pow(2)
    return sign * magnitude
}