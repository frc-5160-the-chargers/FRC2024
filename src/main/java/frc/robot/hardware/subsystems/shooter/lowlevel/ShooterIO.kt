package frc.robot.hardware.subsystems.shooter.lowlevel

import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.robot.hardware.subsystems.GenericIntakeIO

// handles logging and replay for the shooter subsystem
val ShooterLog = LoggableInputsProvider("Shooter")


/**
 * Represents the low level component of the end effector.
 */
interface ShooterIO: GenericIntakeIO {
    val hasNote: Boolean
    val hasNoteDetector: Boolean
}