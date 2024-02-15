package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun runGroundIntake(
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntake
): Command = buildCommand {
    addRequirements(shooter, groundIntake, pivot)

    +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

    loop{
        shooter.intake(-0.2)
        groundIntake.intake(0.7)
    }
}