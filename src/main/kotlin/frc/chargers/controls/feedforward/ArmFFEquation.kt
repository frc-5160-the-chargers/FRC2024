@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularAcceleration
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import edu.wpi.first.math.controller.ArmFeedforward
import frc.chargers.utils.math.units.VoltagePerAngle
import frc.chargers.utils.math.units.VoltagePerAngularAcceleration
import frc.chargers.utils.math.units.VoltagePerAngularVelocity

/**
 * Constructs an [ArmFFEquation] with SI value gains.
 */
fun ArmFFEquation(kS: Double, kG: Double, kV: Double, kA: Double = 0.0) =
    ArmFFEquation(
        Voltage(kS),
        VoltagePerAngle(kG),
        VoltagePerAngularVelocity(kV),
        VoltagePerAngularAcceleration(kA)
    )


/**
 * Represents a feedforward equation that characterizes a
 * angular velocity targeting arm.
 *
 * @see ArmFeedforward
 */
class ArmFFEquation(
    val kS: Voltage,
    val kG: VoltagePerAngle,
    val kV: VoltagePerAngularVelocity,
    val kA: VoltagePerAngularAcceleration = VoltagePerAngularAcceleration(0.0)
) {
    private val baseFF = ArmFeedforward(kS.siValue, kG.siValue, kV.siValue, kA.siValue)

    fun calculate(
        position: Angle,
        velocity: AngularVelocity,
        acceleration: AngularAcceleration = AngularAcceleration(0.0)
    ): Voltage = Voltage(baseFF.calculate(position.siValue, velocity.siValue, acceleration.siValue))

    // arm ff does not have plant inversion calculation due to problem optimization reasons(apparently)
}