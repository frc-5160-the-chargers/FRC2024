package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.robot.hardware.subsystems.GenericIntakeIO

// handles logging + replay for ground intake
val GroundIntakeLog = LoggableInputsProvider("GroundIntake")

interface GroundIntakeIO: GenericIntakeIO {
    val conveyorVoltage: Voltage
    val conveyorCurrent: Current
    val conveyorSpeed: AngularVelocity

    fun setConveyorVoltage(voltage: Voltage)
}