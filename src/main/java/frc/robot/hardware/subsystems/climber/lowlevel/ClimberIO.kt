@file:Suppress("unused")
package frc.robot.hardware.subsystems.climber.lowlevel

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider


val ClimberLog = LoggableInputsProvider("Climber")

interface ClimberIO {
    val leftVoltage: Voltage
    val rightVoltage: Voltage

    val leftSpeed: AngularVelocity
    val rightSpeed: AngularVelocity

    fun setVoltages(leftVoltage: Voltage, rightVoltage: Voltage)
}