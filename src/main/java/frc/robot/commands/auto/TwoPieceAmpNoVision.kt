package frc.robot.commands.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInAmp
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

@Suppress("unused")
fun twoPieceAmpNoVision(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("2pAmpGrab")
    }

    +shootInAmp(pivot, shooter,0.3)

    runParallelUntilFirstCommandFinishes{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpGrab"))

        +runGroundIntake(shooter, pivot, groundIntake)
    }

    runParallelUntilFirstCommandFinishes{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpScore"))

        +pivot.setAngleCommand(PivotAngle.AMP)
    }

    +shootInAmp(pivot, shooter,0.3)

    +basicTaxi(
        drivetrain, shooter = shooter, groundIntake = groundIntake
    )
}