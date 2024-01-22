@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.chargers.utils.math.units.VoltagePerAcceleration
import frc.chargers.utils.math.units.VoltagePerVelocity

/**
 * Constructs a [LinearMotorFFEquation] with SI value gains.
 */
fun ElevatorFFEquation(kS: Double, kG: Double, kV: Double, kA: Double = 0.0) =
    ElevatorFFEquation(
        Voltage(kS),
        Voltage(kG),
        VoltagePerVelocity(kV),
        VoltagePerAcceleration(kA)
    )


/**
 * Represents a feedforward equation that characterizes a
 * linear velocity targeting mechanism with no gravity.
 *
 * This can include flywheels or drivetrain modules.
 *
 * This class inherits the (AngularVelocity) -> Voltage
 * and (AngularVelocity, AngularAcceleration) -> Voltage function types,
 * making it callable as a function.
 *
 * @see SimpleMotorFeedforward
 */
class ElevatorFFEquation(
    val kS: Voltage,
    val kG: Voltage,
    val kV: VoltagePerVelocity,
    val kA: VoltagePerAcceleration = VoltagePerAcceleration(0.0)
): (Velocity, Acceleration) -> Voltage, (Velocity) -> Voltage {

    private val baseFF = ElevatorFeedforward(kS.siValue, kG.siValue, kV.siValue, kA.siValue)

    override operator fun invoke(
        velocity: Velocity,
        acceleration: Acceleration
    ): Voltage = Voltage( baseFF.calculate(velocity.siValue, acceleration.siValue) )

    override operator fun invoke(velocity: Velocity): Voltage =
        invoke(velocity, Acceleration(0.0))
}