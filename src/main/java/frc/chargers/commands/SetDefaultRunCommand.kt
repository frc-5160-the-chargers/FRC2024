package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.internal.LowPriorityInOverloadResolution

/**
 * A utility function for setting the default [Command] of a [Subsystem] to a [RunCommand]
 * in a shorter and easier way.
 *
 * @see Subsystem.setDefaultCommand
 */
fun <S: Subsystem> S.setDefaultRunCommand(
    vararg requirements: Subsystem,
    endBehavior: S.(Boolean) -> Unit = {},
    toRun: S.() -> Unit
){
    defaultCommand =
        RunCommand({toRun()}, this, *requirements)
            .finallyDo{ interrupted -> endBehavior(interrupted) }
}

/**
 * A utility function for setting the default [Command] of a [Subsystem] to a [RunCommand]
 * in a shorter and easier way.
 *
 * @see Subsystem.setDefaultCommand
 */
@LowPriorityInOverloadResolution
fun <S: Subsystem> S.setDefaultRunCommand(
    vararg requirements: Subsystem,
    endBehavior: () -> Unit = {},
    toRun: S.() -> Unit
){
    defaultCommand =
        RunCommand({toRun()}, this, *requirements)
            .finallyDo(endBehavior)
}

