package frc.chargers.utils.math

import edu.wpi.first.math.MathUtil

/**
 * Applies a [deadband] to a double value,
 * which makes it return 0.0 if it is under that deadband value.
 * Useful for controllers.
 */
fun Double.withDeadband(deadband: Double, maxMagnitude: Double? = null): Double =
    if (maxMagnitude == null) {
        MathUtil.applyDeadband(this, deadband)
    }else{
        MathUtil.applyDeadband(this, deadband, maxMagnitude)
    }