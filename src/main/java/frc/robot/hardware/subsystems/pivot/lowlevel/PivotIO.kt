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
    val angle: Angle
    val appliedCurrent: Current
    val tempCelsius: Double

    /**
     * Sets the voltage of the pivot motor.
     */
    fun setVoltage(voltage: Voltage)

    /**
     * Sets the pivot position with only pid control; no motion profiling at all.
     */
    fun setAngleSetpoint(
        position: Angle,
        pidConstants: PIDConstants,
        ffOutput: Voltage = Voltage(0.0)
    )

    /**
     * Sets the angle of the pivot to a certain value.
     *
     * Simulation does not utilize this function
     */
    fun zeroAngle(target: Angle){
        println("Empty Angle zero impl: set to $target")
    }

    /**
     * Sets the brake mode.
     */
    fun setBrakeMode(shouldBrake: Boolean){}



    // not intended to be overriden
    fun getRioPIDController() = SuperPIDController(
        PIDConstants(0.3,0,0),
        getInput = { angle },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )
}