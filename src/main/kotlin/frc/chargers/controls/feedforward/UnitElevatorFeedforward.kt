@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.ElevatorFeedforward
import frc.chargers.framework.ChargerRobot

/**
 * Represents a feedforward equation that characterizes a
 * linear velocity targeting elevator.
 *
 * @see ElevatorFeedforward
 */
class UnitElevatorFeedforward(
    val kS: Double,
    val kG: Double,
    val kV: Double,
    val kA: Double = 0.0,
    distanceUnit: Distance = meters
) {
    private val toMetersScalar = (meters / distanceUnit).siValue
    private val baseFF = ElevatorFeedforward(kS, kG, kV * toMetersScalar, kA * toMetersScalar)

    operator fun invoke(
        velocity: Velocity,
        acceleration: Acceleration = Acceleration(0.0)
    ): Voltage = Voltage(baseFF.calculate(velocity.siValue, acceleration.siValue))

    @JvmName("calculatePlantInversion")
    operator fun invoke(
        currentTarget: Velocity,
        nextTarget: Velocity,
        dt: Time = ChargerRobot.LOOP_PERIOD
    ): Voltage =
        Voltage(
            if (kV == 0.0 || kS == 0.0){
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