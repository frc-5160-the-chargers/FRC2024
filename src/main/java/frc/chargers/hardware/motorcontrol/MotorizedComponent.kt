package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Represents 1 or a group of motors, with a singular encoder,
 * that accomplish the same objective on the robot.
 *
 * This can include spinning one set of wheels on a shooter,
 * moving the gearbox of an arm,
 * and more.
 */
interface MotorizedComponent {
    val encoder: Encoder

    var appliedVoltage: Voltage

    var hasInvert: Boolean

    val statorCurrent: Current

    fun setPositionSetpoint(
        rawPosition: Angle,
        pidConstants: PIDConstants,
        continuousInput: Boolean = false,
        feedforward: Voltage = 0.volts
    )

    fun setVelocitySetpoint(
        rawVelocity: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    )


    var percentOut: Double
        get() = appliedVoltage.siValue / 12.0
        set(value){ appliedVoltage = value * 12.volts }

    fun stop(){
        appliedVoltage = 0.volts
    }

    fun withFollowers(vararg followers: MotorizedComponent) =
        object: MotorizedComponent{
            override val encoder: Encoder = this@MotorizedComponent.encoder

            override var appliedVoltage: Voltage
                get() = this@MotorizedComponent.appliedVoltage
                set(value) {
                    this@MotorizedComponent.appliedVoltage = value
                    followers.forEach{
                        it.appliedVoltage = value
                    }
                }

            override var hasInvert: Boolean = this@MotorizedComponent.hasInvert
                set(value){
                    if (field != value){
                        this@MotorizedComponent.hasInvert = !this@MotorizedComponent.hasInvert
                        followers.forEach{
                            it.hasInvert = !it.hasInvert
                        }
                        field = value
                    }
                }

            override val statorCurrent: Current get() = this@MotorizedComponent.statorCurrent

            override fun setPositionSetpoint(rawPosition: Angle, pidConstants: PIDConstants, continuousInput: Boolean, feedforward: Voltage) {
                this@MotorizedComponent.setPositionSetpoint(rawPosition, pidConstants, continuousInput, feedforward)
                followers.forEach{
                    it.setPositionSetpoint(rawPosition, pidConstants, continuousInput, feedforward)
                }
            }

            override fun setVelocitySetpoint(rawVelocity: AngularVelocity, pidConstants: PIDConstants, feedforward: Voltage) {
                this@MotorizedComponent.setVelocitySetpoint(rawVelocity, pidConstants, feedforward)
                followers.forEach{
                    it.setVelocitySetpoint(rawVelocity, pidConstants, feedforward)
                }
            }
        }
}