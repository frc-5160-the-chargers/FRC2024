@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.controller.ArmFeedforward


/**
 * Represents a feedforward equation that characterizes a
 * angular velocity targeting arm.
 *
 * @see ArmFeedforward
 */
class UnitArmFeedforward(
    val kS: Double,
    val kG: Double,
    val kV: Double,
    val kA: Double = 0.0,
    angleUnit: Angle = radians
) {
    private val toRadianScalar = (radians / angleUnit).siValue
    private val baseFF = ArmFeedforward(kS, kG * toRadianScalar, kV * toRadianScalar, kA * toRadianScalar)

    operator fun invoke(
        position: Angle,
        velocity: AngularVelocity,
        acceleration: AngularAcceleration = AngularAcceleration(0.0)
    ): Voltage = Voltage(baseFF.calculate(position.siValue, velocity.siValue, acceleration.siValue))

    // arm ff does not have plant inversion calculation due to problem optimization reasons(apparently)
}