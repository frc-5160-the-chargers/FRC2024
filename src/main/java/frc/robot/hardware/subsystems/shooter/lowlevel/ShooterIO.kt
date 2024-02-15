package frc.robot.hardware.subsystems.shooter.lowlevel

import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.robot.hardware.subsystems.GenericIntakeIO

// Uses custom wrapper over advantagekit which utilizes property delegates
val ShooterLog = LoggableInputsProvider("Shooter")


/**
 * Represents the low level component of the end effector.
 */
interface ShooterIO: GenericIntakeIO {
    val hasGamepiece: Boolean
    val hasBeamBreakSensor: Boolean
}