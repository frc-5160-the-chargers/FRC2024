package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.quantities.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.controls.pid.PIDConstants

val PivotLog = LoggableInputsProvider("Pivot")

interface PivotIO {
    val appliedVoltage: Voltage
    val position: Angle
    val appliedCurrent: Current
    val tempCelsius: Double

    fun setVoltage(voltage: Voltage)

    fun setPosition(
        position: Angle,
        pidConstants: PIDConstants,
        ffOutput: Voltage = Voltage(0.0)
    )
}