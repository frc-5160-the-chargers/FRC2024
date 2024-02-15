package frc.robot.hardware.subsystems

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage


/**
 * Represents the generic low level componenet of an intake.
 */
interface GenericIntakeIO {
    val intakeVoltages: List<Voltage>
    val intakeSpeeds: List<AngularVelocity>
    val intakeCurrents: List<Current>
    val intakeTemps: List<Double>

    fun intake(voltage: Voltage)
}