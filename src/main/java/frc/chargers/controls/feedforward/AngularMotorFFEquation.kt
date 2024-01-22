@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.AngularAcceleration
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.chargers.utils.math.units.VoltagePerAngularAcceleration
import frc.chargers.utils.math.units.VoltagePerAngularVelocity

/**
 * Constructs an [AngularMotorFFEquation] with SI value gains.
 */
fun AngularMotorFFEquation(kS: Double, kV: Double, kA: Double = 0.0) =
    AngularMotorFFEquation(
        Voltage(kS),
        VoltagePerAngularVelocity(kV),
        VoltagePerAngularAcceleration(kA)
    )


/**
 * Represents a feedforward equation that characterizes an
 * angular velocity targeting mechanism with no gravity.
 *
 * This can include flywheels or drivetrain modules.
 *
 * This class is primarily called as a function, utilizing the
 *
 * @see SimpleMotorFeedforward
 */
class AngularMotorFFEquation(
    val kS: Voltage,
    val kV: VoltagePerAngularVelocity,
    val kA: VoltagePerAngularAcceleration = VoltagePerAngularAcceleration(0.0)
): (AngularVelocity, AngularAcceleration) -> Voltage, (AngularVelocity) -> Voltage {

    private val baseFF = SimpleMotorFeedforward(kS.siValue, kV.siValue, kA.siValue)

    override operator fun invoke(
        velocity: AngularVelocity,
        acceleration: AngularAcceleration
    ): Voltage = Voltage( baseFF.calculate(velocity.siValue, acceleration.siValue) )

    override operator fun invoke(velocity: AngularVelocity): Voltage =
        invoke(velocity, AngularAcceleration(0.0))

    fun toLinear(gearRatio: Double, wheelRadius: Length): LinearMotorFFEquation =
        LinearMotorFFEquation(
            kS.siValue,
            (kV / gearRatio / wheelRadius).siValue,
            kA.siValue / gearRatio / wheelRadius.siValue
        )
}