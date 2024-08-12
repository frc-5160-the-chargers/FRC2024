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
 * @see SimpleMotorFeedforward
 */
class LinearMotorFFEquation(
    val kS: Voltage,
    val kV: VoltagePerVelocity,
    val kA: VoltagePerAcceleration = VoltagePerAcceleration(0.0)
) {
    private val baseFF = SimpleMotorFeedforward(kS.siValue, kV.siValue, kA.siValue)

    fun calculate(
        velocity: Velocity,
        acceleration: Acceleration = Acceleration(0.0)
    ): Voltage = Voltage(baseFF.calculate(velocity.siValue, acceleration.siValue))

    @JvmName("calculatePlantInversion")
    fun calculate(
        currentTarget: Velocity,
        nextTarget: Velocity,
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

    fun toAngular(gearRatio: Double, wheelRadius: Length): AngularMotorFFEquation =
        AngularMotorFFEquation(
            kS.siValue,
            kV.siValue * gearRatio * wheelRadius.siValue,
            kA.siValue * gearRatio * wheelRadius.siValue
        )
}