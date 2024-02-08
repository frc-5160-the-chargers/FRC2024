package frc.robot.hardware.subsystems.shooter.lowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.controls.pid.PIDConstants

// Uses custom wrapper over advantagekit which utilizes property delegates
val ShooterLog = LoggableInputsProvider("Shooter")


/**
 * Represents the low level component of the end effector.
 */
interface ShooterIO {
    val hasGamepiece: Boolean
    val hasBeamBreakSensor: Boolean

    val intakeVoltages: List<Voltage>
    val intakeSpeeds: List<AngularVelocity>
    val intakeCurrents: List<Current>
    val intakeTemps: List<Double>

    fun setVoltage(voltage: Voltage)


    val pivotVoltage: Voltage
    val pivotPosition: Angle
    val pivotCurrent: Current
    val pivotTemp: Double

    fun setPivotVoltage(voltage: Voltage)

    fun setPivotPosition(
        position: Angle,
        pidConstants: PIDConstants,
        ffOutput: Voltage = Voltage(0.0)
    )
}