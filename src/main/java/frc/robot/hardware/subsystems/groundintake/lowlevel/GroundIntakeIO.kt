package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider


val GroundIntakeLog = LoggableInputsProvider("GroundIntake")

/**
 * Standard order for lists is top motor, then bottom motor(if available).
 */
interface GroundIntakeIO {
    val intakeVoltages: List<Voltage>
    val intakeCurrents: List<Current>
    val intakeSpeeds: List<AngularVelocity>

    val conveyorVoltage: Voltage
    val conveyorCurrent: Current
    val conveyorSpeed: AngularVelocity

    fun intake(voltage: Voltage)
}