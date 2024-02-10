package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.grabGamepiece
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

@Suppress("unused")
fun twoPieceAmpNoVision(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("2pAmpGrab")
    }

    +onePieceAmp(shooter)

    +grabGamepiece(
        path = PathPlannerPath.fromPathFile("2pAmpGrab"),
        noteDetector = null,
        drivetrain, shooter, groundIntake
    )

    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("2pAmpScore"), PATHFIND_CONSTRAINTS)

        +shooter.setAngleCommand(PivotAngle.AMP)
    }

    loopFor(1.seconds){
        shooter.outtake(0.3)
    }

    runOnce{
        shooter.setIdle()
    }

    +basicTaxi(
        drivetrain, shooter = shooter, groundIntake = groundIntake
    )
}