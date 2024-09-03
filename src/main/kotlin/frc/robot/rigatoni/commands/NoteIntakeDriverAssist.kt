package frc.robot.rigatoni.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.rigatoni.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.rigatoni.inputdevices.DriverController
import frc.robot.rigatoni.subsystems.NoteObserver
import kotlin.math.abs
import kotlin.math.max

fun noteIntakeDriverAssist(
    drivetrain: EncoderHolonomicDrivetrain,
    noteObserver: NoteObserver
): Command = buildCommand(log = true) {
    fun getNotePursuitSpeed(txValue: Double): Double {
        val swerveOutput = DriverController.swerveOutput
        return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - txValue / 50.0) // scales based off of the vision target error
    }

    fun shouldStartPursuit(): Boolean {
        val currentState = noteObserver.state
        return currentState is NoteObserver.State.NoteDetected &&
                currentState.distanceToNote <= ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
    }

    require(drivetrain)

    // regular drive occurs until suitable target found
    loopUntil(::shouldStartPursuit){
        drivetrain.swerveDrive(DriverController.swerveOutput, !DriverController.shouldDisableFieldRelative)
    }

    loop{
        // drives back to grab note
        val currentState = noteObserver.state
        if (currentState is NoteObserver.State.NoteDetected){
            drivetrain.swerveDrive(getNotePursuitSpeed(currentState.tx), 0.0, 0.0, fieldRelative = false)
        }else{
            drivetrain.swerveDrive(getNotePursuitSpeed(0.0), 0.0, 0.0, fieldRelative = false)
        }
    }
}