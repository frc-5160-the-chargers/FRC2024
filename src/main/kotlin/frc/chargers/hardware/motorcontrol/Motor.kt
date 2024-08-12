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
    val encoder: Encoder

    val statorCurrent: Current

    var appliedVoltage: Voltage

    var hasInvert: Boolean

    fun setBrakeMode(shouldBrake: Boolean)

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

    fun withFollowers(vararg followers: Motor) =
        object: Motor {
            override val encoder: Encoder = this@Motor.encoder

            // runs a function for all the included motors ; including followers.
            inline fun runForAllMotors(function: (Motor) -> Unit){
                function(this@Motor)
                followers.forEach(function)
            }

            override var appliedVoltage: Voltage
                get() = this@Motor.appliedVoltage
                set(value) {
                    runForAllMotors{ it.appliedVoltage = value }
                }

            override var hasInvert: Boolean = this@Motor.hasInvert
                set(value){
                    if (field != value){
                        field = value
                        runForAllMotors{ it.hasInvert = !it.hasInvert }
                    }
                }

            override val statorCurrent: Current get() = this@Motor.statorCurrent

            override fun setBrakeMode(shouldBrake: Boolean) {
                runForAllMotors{ it.setBrakeMode(shouldBrake) }
            }

            override fun setPositionSetpoint(rawPosition: Angle, pidConstants: PIDConstants, continuousInput: Boolean, feedforward: Voltage) {
                runForAllMotors{ it.setPositionSetpoint(rawPosition, pidConstants, continuousInput, feedforward) }
            }

            override fun setVelocitySetpoint(rawVelocity: AngularVelocity, pidConstants: PIDConstants, feedforward: Voltage) {
                runForAllMotors{ it.setVelocitySetpoint(rawVelocity, pidConstants, feedforward) }
            }
        }
}