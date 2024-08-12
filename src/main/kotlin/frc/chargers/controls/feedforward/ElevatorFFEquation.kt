@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ElevatorFeedforward
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.units.VoltagePerAcceleration
import frc.chargers.utils.math.units.VoltagePerVelocity

/**
 * Constructs a [ElevatorFFEquation] with SI value gains.
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
 * linear velocity targeting elevator.
 *
 * @see ElevatorFeedforward
 */
class ElevatorFFEquation(
    val kS: Voltage,
    val kG: Voltage,
    val kV: VoltagePerVelocity,
    val kA: VoltagePerAcceleration = VoltagePerAcceleration(0.0)
) {
    private val baseFF = ElevatorFeedforward(kS.siValue, kG.siValue, kV.siValue, kA.siValue)

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
}