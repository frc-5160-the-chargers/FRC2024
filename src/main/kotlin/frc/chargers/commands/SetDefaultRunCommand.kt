package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem

/**
 * A utility function for setting the default [Command] of a [Subsystem] to a [RunCommand]
 * in a shorter and easier way.
 *
 * @see Subsystem.setDefaultCommand
 */
fun Subsystem.setDefaultRunCommand(
    vararg requirements: Subsystem,
    toRun: () -> Unit
){
    this.defaultCommand =
        RunCommand(toRun, this, *requirements)
            .withName("Default Command of " + this::class.simpleName)
}


