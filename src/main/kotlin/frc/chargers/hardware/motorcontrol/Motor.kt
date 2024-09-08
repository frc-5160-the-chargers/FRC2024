@file:Suppress("unused")
package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Represents 1 or a group of motors, with a singular encoder,
 * that accomplish the same objective on the robot.
 *
 * This can include spinning one set of wheels on a shooter,
 * moving the gearbox of an arm,
 * and more.
 */
interface Motor {
    /**
     * The encoder of the motor.
     */
    val encoder: Encoder

    /**
     * A property that either fetches the current applied voltage,
     * or sets its value.
     */
    var appliedVoltage: Voltage

    /**
     * The motor's stator current.
     */
    val statorCurrent: Current

    /**
     * Whether the motor is inverted or not.
     */
    val inverted: Boolean

    /**
     * Uses onboard closed-loop control to set the position of a motor.
     * This function will only work if you set the positionPID property
     * via the [configure] function.
     */
    fun setPositionSetpoint(position: Angle, feedforward: Voltage = 0.volts)

    /**
     * Uses onboard closed-loop control to set the velocity of a motor.
     * This function will only work if you set the velocityPID property
     * via the [configure] function.
     */
    fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage)

    /**
     * Configures the motor with a couple of standard settings.
     *
     * This is intended to either be an isolated call or an inline call;
     * with the [Motor] being returned for convenient inline configuration.
     *
     * Note: a [positionUpdateRate] or [velocityUpdateRate] of 0.hertz
     * will disable updating for those signals.
     */
    fun configure(
        inverted: Boolean? = null,
        brakeWhenIdle: Boolean? = null,
        rampRate: Time? = null,
        statorCurrentLimit: Current? = null,
        followerMotors: List<Motor> = listOf(),
        positionUpdateRate: Frequency? = null,
        velocityUpdateRate: Frequency? = null,
        optimizeUpdateRate: Boolean? = null,

        gearRatio: Double? = null,
        startingPosition: Angle? = null,
        positionPID: PIDConstants? = null,
        velocityPID: PIDConstants? = null,
        continuousInput: Boolean? = null
    ): Motor


    var speed: Double
        get() = (appliedVoltage / 12.volts).siValue
        set(value) {
            appliedVoltage = value * 12.volts
        }

    /**
     * Stops the motor.
     */
    fun stop() { appliedVoltage = 0.volts }
}