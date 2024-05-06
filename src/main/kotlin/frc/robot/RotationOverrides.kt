package frc.robot

import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.RotationOverride
import frc.robot.subsystems.NoteObserver

object RotationOverrides{
    fun aimToNote(noteDetector: NoteObserver): RotationOverride =
        AimToObjectRotationOverride(
            getCrosshairOffset = {
                val currentState = noteDetector.state
                if (currentState is NoteObserver.State.NoteDetected) currentState.tx else null
            },
            PIDConstants(0.05, 0, 0.001)
        )
}