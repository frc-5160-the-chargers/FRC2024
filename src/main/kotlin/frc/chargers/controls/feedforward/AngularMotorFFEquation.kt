@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.chargers.framework.ChargerRobot
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
){
    private val baseFF = SimpleMotorFeedforward(kS.siValue, kV.siValue, kA.siValue)

    fun calculate(
        velocity: AngularVelocity,
        acceleration: AngularAcceleration = AngularAcceleration(0.0)
    ): Voltage = Voltage(baseFF.calculate(velocity.siValue, acceleration.siValue))

    @JvmName("calculatePlantInversion")
    fun calculate(
        currentTarget: AngularVelocity,
        nextTarget: AngularVelocity,
        dt: Time = ChargerRobot.LOOP_PERIOD
    ): Voltage =
        Voltage(
            if (kV.siValue == 0.0 || kS.siValue == 0.0){
                0.0
            }else{
                baseFF.calculate(
                    currentTarget.siValue,
                    nextTarget.siValue,
                    dt.inUnit(seconds)
                )
            }
        )

    fun toLinear(gearRatio: Double, wheelRadius: Length): LinearMotorFFEquation =
        LinearMotorFFEquation(
            kS.siValue,
            (kV / gearRatio / wheelRadius).siValue,
            kA.siValue / gearRatio / wheelRadius.siValue
        )
}