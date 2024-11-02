package frc.chargers.utils

import choreo.auto.AutoLoop
import choreo.auto.AutoTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.Cmd
import java.util.Optional

// tbd until its renamed
typealias AutoRoutine = AutoLoop

fun AutoRoutine.subsystemsAvailable(vararg subsystems: Subsystem) =
    Trigger(this.loop) { subsystems.all { it.currentCommand == it.defaultCommand } }
        .and(this.enabled())

val AutoRoutine.enabled: Trigger get() = this.enabled()
val AutoTrajectory.done: Trigger get() = this.done()
val AutoTrajectory.active: Trigger get() = this.active()
val AutoTrajectory.inactive: Trigger get() = this.inactive()
//val AutoTrajectory.cmd: Command get() = this.cmd()

fun Optional<Pose2d>.orElseKill(routine: AutoRoutine): Pose2d {
    if (this.isPresent) return this.get()
    routine.kill()
    return Pose2d()
}

fun testAuto(drivetrain: EncoderHolonomicDrivetrain): Command {
    drivetrain.choreoApi.apply {
        val routine = newLoop("5pAuto")
        val path1 = trajectory("Something", routine)
        val path2 = trajectory("Something", routine)
        val path3 = trajectory("Something", routine)

        routine.enabled.onTrue(
            Cmd.runOnce { drivetrain.resetPose(path1.initialPose.orElseKill(routine)) }
                .andThen(path1.cmd())
        )
        path1.atTime(0.35).onTrue()
        path1.done.onTrue(path3.cmd())

        path2.atTime(0.36).onTrue()
        path2.active.whileTrue()
        path2.done.onTrue(path3.cmd())

        return routine.cmd()
    }
}

fun testAutoTwo(drivetrain: EncoderHolonomicDrivetrain): Command {
    drivetrain.choreoApi.apply {
        return Cmd.sequence(
            drivetrain.resetPoseThenPathCommand("Hello"),
            drivetrain.pathCommand("Hello").apply {
                atTime(0.35).onTrue(intake())
            },
            drivetrain.pathCommand("Hello").apply {
                atTime(0.35).onTrue()
            }
            drivetrain.pathCommand("Final")
                .whenDoneAnd
        )
    }
}