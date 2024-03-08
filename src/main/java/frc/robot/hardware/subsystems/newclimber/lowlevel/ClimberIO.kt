@file:Suppress("unused")
package frc.robot.hardware.subsystems.newclimber.lowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider


val ClimberLog = LoggableInputsProvider("Climber")

interface ClimberIO {
    val leftVoltage: Voltage
    val rightVoltage: Voltage

    val leftSpeed: AngularVelocity
    val rightSpeed: AngularVelocity

    val leftPosition: Angle
    val rightPosition: Angle


    fun setLeftVoltage(voltage: Voltage)
    fun setRightVoltage(voltage: Voltage)
}