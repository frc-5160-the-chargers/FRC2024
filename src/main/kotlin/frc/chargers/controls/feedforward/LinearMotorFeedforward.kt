@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.chargers.framework.ChargerRobot

/**
 * Represents a feedforward equation that characterizes a
 * linear velocity targeting mechanism with no gravity.
 *
 * This can include flywheels or drivetrain modules.
 *
 * @see SimpleMotorFeedforward
 */
class LinearMotorFeedforward(
    val kS: Double,
    val kV: Double,
    val kA: Double = 0.0,
    val distanceUnit: Distance = meters
) {
    private val toMetersScalar = (meters / distanceUnit).siValue
    private val baseFF = SimpleMotorFeedforward(kS, kV * toMetersScalar, kA * toMetersScalar)

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

    fun toAngular(gearRatio: Double, wheelRadius: Length): AngularMotorFeedforward =
        AngularMotorFeedforward(
            kS,
            kV * toMetersScalar / gearRatio * wheelRadius.inUnit(meters),
            kA * toMetersScalar / gearRatio * wheelRadius.inUnit(meters),
            radians
        )
}