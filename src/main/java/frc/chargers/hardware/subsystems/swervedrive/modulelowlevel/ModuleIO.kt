package frc.chargers.hardware.subsystems.swervedrive.modulelowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController

/**
 * Represents the basic(low level) control component of a single swerve module.
 *
 * Further optimizations and control mechanisms are done within the drivetrain class.
 */
abstract class ModuleIO {
    abstract val direction: Angle
    abstract val turnSpeed: AngularVelocity

    abstract val speed: AngularVelocity
    abstract val wheelTravel: Angle

    abstract val driveCurrent: Current
    abstract val turnCurrent: Current

    abstract val turnVoltage: Voltage
    abstract val driveVoltage: Voltage

    abstract fun setTurnVoltage(voltage: Voltage)

    abstract fun setDriveVoltage(voltage: Voltage)

    open fun setDirectionSetpoint(
        direction: Angle,
        pidConstants: PIDConstants,
        feedforward: Voltage = Voltage(0.0)
    ){
        defaultAzimuthController.constants = pidConstants
        setTurnVoltage(defaultAzimuthController.calculateOutput(direction) + feedforward)
    }

    open fun setSpeedSetpoint(
        velocity: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    ){
        defaultVelocityController.constants = pidConstants
        setDriveVoltage(defaultVelocityController.calculateOutput(velocity) + feedforward)
    }



    fun disableDefaultControllers(){
        defaultAzimuthController.disableSelfSustain()
        defaultVelocityController.disableSelfSustain()
    }

    private val defaultAzimuthController = SuperPIDController(
        PIDConstants(0,0,0),
        getInput = { direction },
        target = Angle(0.0),
        outputRange = (-12).volts..12.volts,
        continuousInputRange = 0.degrees..360.degrees,
        selfSustain = true
    )

    private val defaultVelocityController = SuperPIDController(
        PIDConstants(0,0,0),
        getInput = { speed },
        target = AngularVelocity(0.0),
        outputRange = (-12).volts..12.volts,
        selfSustain = true
    )
}





