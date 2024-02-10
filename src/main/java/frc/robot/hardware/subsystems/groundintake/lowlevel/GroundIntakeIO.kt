package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider


val GroundIntakeLog = LoggableInputsProvider("GroundIntake")

interface GroundIntakeIO {
    val measuredVoltages: List<Voltage>
    val measuredCurrents: List<Current>
    val measuredSpeeds: List<AngularVelocity>

    fun intake(voltage: Voltage)
}