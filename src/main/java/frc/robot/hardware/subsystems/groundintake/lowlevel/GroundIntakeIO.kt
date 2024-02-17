package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Current
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.robot.hardware.subsystems.GenericIntakeIO

// handles logging + replay for ground intake
val GroundIntakeLog = LoggableInputsProvider("GroundIntake")

/**
 * Standard order for lists is top motor, then bottom motor(if available).
 */
interface GroundIntakeIO: GenericIntakeIO {
    val conveyorVoltage: Voltage
    val conveyorCurrent: Current
    val conveyorSpeed: AngularVelocity
}