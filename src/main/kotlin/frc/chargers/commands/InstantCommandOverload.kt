package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
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
fun InstantCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand =
    InstantCommand(toRun, *subsystems)