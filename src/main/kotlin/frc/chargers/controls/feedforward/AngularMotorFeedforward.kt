@file:Suppress("unused")
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.controller.SimpleMotorFeedforward


/**
 * Represents a feedforward equation that characterizes an
 * angular velocity targeting mechanism with no gravity.
 *
 * This can include flywheels or drivetrain modules.
 *
 * @see SimpleMotorFeedforward
 */
class AngularMotorFeedforward(
    val kS: Double,
    val kV: Double,
    val kA: Double = 0.0,
    val angleUnit: Angle = radians
){
    private val toRadianScalar = (radians / angleUnit).siValue
    private val baseFF = SimpleMotorFeedforward(kS, kV * toRadianScalar, kA * toRadianScalar)

    operator fun invoke(
        velocity: AngularVelocity,
        acceleration: AngularAcceleration = AngularAcceleration(0.0)
    ): Voltage = Voltage(baseFF.calculate(velocity.siValue, acceleration.siValue))

    @JvmName("calculatePlantInversion")
    operator fun invoke(
        currentTarget: AngularVelocity,
        nextTarget: AngularVelocity,
        dt: Time = 0.02.seconds
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

    fun toLinear(gearRatio: Double, wheelRadius: Length): LinearMotorFeedforward =
        LinearMotorFeedforward(
            kS,
            kV * toRadianScalar * gearRatio / wheelRadius.inUnit(meters),
            kA * toRadianScalar * gearRatio / wheelRadius.inUnit(meters),
            meters
        )
}