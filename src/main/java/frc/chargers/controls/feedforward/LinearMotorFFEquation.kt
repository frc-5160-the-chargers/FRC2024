@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")

package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.units.VoltagePerAcceleration
import frc.chargers.utils.math.units.VoltagePerVelocity


/**
 * Constructs a [LinearMotorFFEquation] with SI value gains.
 */
fun LinearMotorFFEquation(kS: Double, kV: Double, kA: Double = 0.0) =
    LinearMotorFFEquation(
        Voltage(kS),
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
class LinearMotorFFEquation(
    val kS: Voltage,
    val kV: VoltagePerVelocity,
    val kA: VoltagePerAcceleration = VoltagePerAcceleration(0.0)
): (Velocity, Acceleration) -> Voltage, (Velocity) -> Voltage {

    private val baseFF = SimpleMotorFeedforward(kS.siValue, kV.siValue, kA.siValue)

    override operator fun invoke(
        velocity: Velocity,
        acceleration: Acceleration
    ): Voltage = Voltage( baseFF.calculate(velocity.siValue, acceleration.siValue) )

    override operator fun invoke(velocity: Velocity): Voltage =
        invoke(velocity, Acceleration(0.0))

    fun toAngular(gearRatio: Double, wheelRadius: Length): AngularMotorFFEquation =
        AngularMotorFFEquation(
            kS.siValue,
            kV.siValue * gearRatio * wheelRadius.siValue,
            kA.siValue * gearRatio * wheelRadius.siValue
        )

    fun calculatePlantInversion(currentTarget: Velocity, nextTarget: Velocity): Voltage =
        Voltage(
            if (kV.siValue == 0.0 || kS.siValue == 0.0){
                0.0
            }else{
                baseFF.calculate(
                    currentTarget.siValue,
                    nextTarget.siValue,
                    ChargerRobot.LOOP_PERIOD.inUnit(seconds)
                )
            }
        )
}