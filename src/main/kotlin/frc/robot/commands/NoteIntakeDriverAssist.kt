package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.inputdevices.DriverController
import frc.robot.subsystems.NoteObserver
import kotlin.math.abs
import kotlin.math.max

fun noteIntakeDriverAssist(
    drivetrain: EncoderHolonomicDrivetrain,
    noteObserver: NoteObserver
): Command = buildCommand(logIndividualCommands = true) {
    fun getNotePursuitSpeed(txValue: Double): Double{
        val swerveOutput = DriverController.swerveOutput
        return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - txValue / 50.0) // scales based off of the vision target error
    }

    // regular drive occurs until suitable target found
    loopUntil({
        val currentState = noteObserver.state
        currentState is NoteObserver.State.NoteDetected &&
        currentState.distanceToNote <= ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
    }, drivetrain){
        drivetrain.swerveDrive(DriverController.swerveOutput, !DriverController.shouldDisableFieldRelative)
    }

    loop(drivetrain){
        // drives back to grab note
        val currentState = noteObserver.state
        if (currentState is NoteObserver.State.NoteDetected){
            drivetrain.swerveDrive(getNotePursuitSpeed(currentState.tx), 0.0, 0.0, fieldRelative = false)
        }else{
            drivetrain.swerveDrive(getNotePursuitSpeed(0.0), 0.0, 0.0, fieldRelative = false)
        }
    }
}