package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun runGroundIntake(
    shooter: Shooter,
    groundIntake: GroundIntake
): Command = buildCommand {
    addRequirements(shooter, groundIntake)

    +shooter.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

    loop{
        shooter.intake(-0.2)
        groundIntake.intake(0.7)
    }
}