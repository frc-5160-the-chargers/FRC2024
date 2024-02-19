package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController

// handles logging and replay for the pivot subsystem.
val PivotLog = LoggableInputsProvider("Pivot")

/**
 * Represents the low level component of the pivot subsystem.
 */
interface PivotIO {
    val appliedVoltage: Voltage
    val position: Angle
    val appliedCurrent: Current
    val tempCelsius: Double

    fun setVoltage(voltage: Voltage)

    /**
     * Sets the pivot position with only pid control; no motion profiling at all.
     */
    fun setPositionSetpoint(
        position: Angle,
        pidConstants: PIDConstants,
        ffOutput: Voltage = Voltage(0.0)
    )

    // not intended to be overriden
    fun getRioPIDController() = SuperPIDController(
        PIDConstants(0.3,0,0),
        getInput = { position },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )
}