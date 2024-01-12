@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.motorcontrol


import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.chargers.constants.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder

/**
 * Represents a motor controller that supports an encoder.
 *
 * Examples of motor controllers supporting encoders are
 * the SparkMax, Talon FX, and Talon SRX.
 */
public interface EncoderMotorController : MotorController {
    public val encoder: Encoder
}


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
     * Sets the angular velocity of the motor.
     */
    public fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
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
     * Sets the position of the motor using closed loop control;
     * utilizing the readings of an absolute encoder.
     */
    public fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean = false,
        extraVoltage: Voltage = Voltage(0.0),
        turnEncoder: PositionEncoder,
        motorToEncoderRatio: Double = DEFAULT_GEAR_RATIO,
    ){
        val positionError = motorToEncoderRatio * (target - turnEncoder.angularPosition)

        setAngularPosition(
            encoder.angularPosition + positionError,
            pidConstants,
            continuousWrap,
            extraVoltage
        )
    }
}

