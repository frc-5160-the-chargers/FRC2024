@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.utils.math.inputModulus

/**
 * A utility class that manages closed loop control
 * for a CAN spark device.
 */
internal class SparkPIDHandler(
    motor: com.revrobotics.CANSparkBase,
    private val encoderAdaptor: Encoder
) {

    private val innerController = motor.pidController
    private var currentPIDConstants = PIDConstants(0.0,0.0,0.0)
    private var isCurrentlyWrapping = false


    private fun updateControllerConstants(newConstants: PIDConstants){
        if (currentPIDConstants != newConstants){
            innerController.setP(newConstants.kP,0)
            innerController.setI(newConstants.kI,0)
            innerController.setD(newConstants.kD,0)
            currentPIDConstants = newConstants
        }
    }

    fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage,
        vararg followers: SmartEncoderMotorController
    ) {
        updateControllerConstants(pidConstants)
        innerController.setReference(
            target.siValue,
            com.revrobotics.CANSparkBase.ControlType.kVelocity,
            0,
            feedforward.inUnit(volts)
        )
        followers.forEach{
            it.setAngularVelocity(target, pidConstants, feedforward)
        }
    }

    fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean,
        extraVoltage: Voltage,
        vararg followers: SmartEncoderMotorController
    ) {
        if (continuousWrap != isCurrentlyWrapping){
            if (continuousWrap){
                innerController.positionPIDWrappingEnabled = true
                innerController.positionPIDWrappingMinInput = -180.degrees.siValue
                innerController.positionPIDWrappingMaxInput = 180.degrees.siValue
            }else{
                innerController.positionPIDWrappingEnabled = false
            }
            isCurrentlyWrapping = continuousWrap
        }
        updateControllerConstants(pidConstants)

        innerController.setReference(
            if (isCurrentlyWrapping){
                target.inputModulus(-180.degrees..180.degrees).siValue
            }else{
                target.siValue
            },
            com.revrobotics.CANSparkBase.ControlType.kPosition,
            0,
            extraVoltage.siValue
        )

        followers.forEach{
            it.setAngularPosition(
                target, pidConstants, continuousWrap, extraVoltage, encoderAdaptor
            )
        }
    }

}