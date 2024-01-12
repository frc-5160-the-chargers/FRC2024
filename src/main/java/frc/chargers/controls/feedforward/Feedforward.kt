@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.dimensions.VelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward

/**
 * Represents a Generic Feedforward.
 * A feedforward is an equation which estimates the Control Effort(I) required
 * to achieve a certain output(O).
 *
 * See [here](https://www.controleng.com/articles/feed-forwards-augment-pid-control/) for an explanation of feedforward, and
 * See [here](https://kotlinlang.org/docs/fun-interfaces.html) for an explanation of Functional/SAM interfaces.
 *
 * Example usage of the Feedforward interface:
 *
 * ```
 * // custom FF definition
 * val ff = Feedforward<AngularVelocityDimension, VoltageDimension>{ Voltage(it.siValue * 2.0) }
 *
 * // WPILib wrapped feedforward
 * // kS is a Voltage, kV is VoltagePerVelocity
 * // and kA is VoltagePerAcceleration
 * val simpleMotorAngularFF: Feedforward<AngularVelocityDimension, VoltageDimension> =
 *      Feedforward(AngularMotorFFConstants(kS, kV, kA))
 * // Alternatively, the fromSI factory function can be used if all units are SI units.
 * val armFF = Feedforward(ArmFFConstants.fromSI(ksDouble, kGDouble, kVDouble, kADouble))
 */
public fun interface Feedforward<I: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>> {
    public fun calculate(value: Quantity<I>): Quantity<O>
}


/**
 * Wraps WPILib's [SimpleMotorFeedforward], adding units support.
 *
 * This function takes in an [AngularVelocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: AngularMotorFFConstants,
    crossinline getTargetAccel: () -> AngularAcceleration = { AngularAcceleration(0.0) }
): Feedforward<AngularVelocityDimension, VoltageDimension>{
    val baseFF = SimpleMotorFeedforward(constants.kS.siValue, constants.kV.siValue, constants.kA.siValue)
    return Feedforward{ targetVel: AngularVelocity ->
        Voltage(
            baseFF.calculate(targetVel.siValue, getTargetAccel().siValue)
        )
    }
}


/**
 * Wraps WPILib's [ArmFeedforward], adding units support.
 *
 * This function takes in an [AngularVelocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: ArmFFConstants,
    crossinline getAngle: () -> Angle,
    crossinline getTargetAccel: () -> AngularAcceleration = { AngularAcceleration(0.0) }
): Feedforward<AngularVelocityDimension, VoltageDimension>{
    val baseFF = ArmFeedforward(constants.kS.siValue, constants.kG.siValue, constants.kV.siValue, constants.kA.siValue)
    return Feedforward{ targetVel: AngularVelocity ->
        Voltage(
            baseFF.calculate(getAngle().siValue, targetVel.siValue, getTargetAccel().siValue)
        )
    }
}

/**
 * Wraps WPILib's [ArmFeedforward], adding units support.
 *
 * This function takes in a [Velocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: LinearMotorFFConstants,
    crossinline getTargetAccel: () -> Acceleration = { Acceleration(0.0) }
): Feedforward<VelocityDimension, VoltageDimension>{
    val baseFF = SimpleMotorFeedforward(constants.kS.siValue, constants.kV.siValue, constants.kA.siValue)
    return Feedforward{ targetVel: Velocity ->
        Voltage(
            baseFF.calculate(targetVel.siValue, getTargetAccel().siValue)
        )
    }
}

/**
 * Wraps WPILib's [ElevatorFeedforward], adding units support.
 *
 * This function takes in a [Velocity] and outputs a [Voltage];
 * providing a [Feedforward] functional interface implementation.
 */
public inline fun Feedforward(
    constants: ElevatorFFConstants,
    crossinline getTargetAccel: () -> AngularAcceleration = { AngularAcceleration(0.0) }
): Feedforward<VelocityDimension, VoltageDimension>{
    val baseFF = ElevatorFeedforward(constants.kS.siValue, constants.kG.siValue, constants.kV.siValue, constants.kA.siValue)
    return Feedforward{ targetVel: Velocity ->
        Voltage(
            baseFF.calculate(targetVel.siValue, getTargetAccel().siValue)
        )
    }
}



