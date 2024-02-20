package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter

fun idleSubsystems(
    drivetrain: EncoderHolonomicDrivetrain? = null,
    shooter: Shooter? = null,
    pivot: Pivot? = null,
    groundIntake: GroundIntakeSerializer? = null
): Command = buildCommand {
    if(drivetrain != null) addRequirements(drivetrain)
    if (shooter != null) addRequirements(shooter)
    if (pivot != null) addRequirements(pivot)
    if (groundIntake != null) addRequirements(groundIntake)

    runOnce{
        shooter?.setIdle()
        pivot?.setIdle()
        groundIntake?.setIdle()
        drivetrain?.stopInX()
    }
}