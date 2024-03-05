package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem

/**
 * A utility function for creating [InstantCommand]s in a more kotlin-friendly way.
 *
 * Because of the lambda function being placed at the end, it can now use kotlin's
 * external lambda block syntax.
 *
 * ```
 * // this
 * val instantCommand = runOnceCommand(drivetrain, arm){
 *      arm.move()
 *      drivetrain.drive()
 * }
 *
 * // instead of this
 * val instantCommand = InstantCommand({arm.move(); drivetrain.drive();}, drivetrain, arm)
 */
fun runOnceCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand =
    InstantCommand(toRun, *subsystems)

/**
 * A utility function for creating [RunCommand]s in a more kotlin-friendly way.
 *
 * Because of the lambda function being placed at the end, it can now use kotlin's
 * external lambda block syntax.
 *
 * ```
 * // this
 * val runCommand = loopCommand(drivetrain, arm){
 *      arm.move()
 *      drivetrain.drive()
 * }
 *
 * // instead of this
 * val runCommand = RunCommand({arm.move(); drivetrain.drive();}, drivetrain, arm)
 */
fun loopCommand(vararg subsystems: Subsystem, toRun: () -> Unit): RunCommand =
    RunCommand(toRun, *subsystems)