package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.idleSubsystems
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

@Suppress("unused")
fun twoNoteAmpNoVision(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
): Command = buildCommand(name = "Two piece amp(no vision)", logIndividualCommands = true){
    addRequirements(drivetrain, shooter, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("AmpGrabG2", useHolonomicPose = true)
    }

    loopFor(0.5.seconds){
        shooter.outtake(0.3)
    }

    runParallelUntilFirstCommandFinishes{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpGrabG2"))

        loop{
            groundIntake.intake(pivot)
        }
    }

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpScoreG2"))

        loopUntil({shooter.hasNote}){
            groundIntake.passToShooter(shooter)
        }
    }

    loopFor(0.5.seconds){
        shooter.outtake(0.3)
    }

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    runParallelUntilAllFinish{
        +basicTaxi(drivetrain)

        +pivot.setAngleCommand(PivotAngle.AMP)
    }
}