@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import frc.chargers.constants.DEFAULT_GEAR_RATIO
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.encoders.PositionEncoder

/**
 * An interface that represents an [EncoderMotorController]
 * that can measure it's recorded temperature, applied current, and applied voltage,
 * as well as run closed loop control on the motor itself.
 */
public interface SmartEncoderMotorController: EncoderMotorController{
    /**
     * The applied current of the motor.
     */
    public val appliedCurrent: Current

    /**
     * The applied voltage of the motor.
     */
    public val appliedVoltage: Voltage

    /**
     * The temperature of the motor.
     */
    public val tempCelsius: Double

    /**
     * Sets the angular velocity of the motor, with a feedforward voltage.
     */
    public fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    )

    /**
     * Sets the position of the motor using closed loop control.
     */
    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean = false,
        extraVoltage: Voltage = Voltage(0.0)
    )





    /**
     * Sets the angular velocity of the motor, with a feedforward equation.
     */
    public fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        ffEquation: (AngularVelocity) -> Voltage
    ) = setAngularVelocity(target, pidConstants, ffEquation(target))

    /**
     * Sets the position of the motor using closed loop control;
     * utilizing the readings of an absolute encoder.
     */
    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean = false,
        extraVoltage: Voltage = Voltage(0.0),
        absoluteEncoder: PositionEncoder,
        /**
         * The motor to encoder ratio.
         * This is usually the gear ratio of the motor,
         * unless if the absolute encoder is on gearing.
         */
        motorToEncoderRatio: Double = DEFAULT_GEAR_RATIO,
    ){
        val positionError = (target - absoluteEncoder.angularPosition) / motorToEncoderRatio

        // remove this in the future
        println("Target position: " + (this.encoder.angularPosition + positionError))

        setAngularPosition(
            this.encoder.angularPosition + positionError,
            pidConstants,
            continuousWrap,
            extraVoltage
        )
    }
}
