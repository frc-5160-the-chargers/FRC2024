package frc.robot.hardware.subsystems

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage


/**
 * Represents the generic low level component of an intake on the robot.
 * This includes the ground intake and the shooter.
 *
 * In general, the standard order for this is
 */
interface GenericIntakeIO {
    val intakeVoltages: List<Voltage>
    val intakeSpeeds: List<AngularVelocity>
    val intakeCurrents: List<Current>
    val intakeTemps: List<Double>

    fun setIntakeVoltage(voltage: Voltage)
}