package frc.robot.commands.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.pathplannerextensions.PathPlannerPaths
import frc.robot.commands.grabGamepiece
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter


@Suppress("unused")
fun sixPieceSpeakerBeta(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,

    noteDetector: ObjectVisionPipeline
): Command = buildCommand {
    // adds a command that makes the drivebase follow a path, while intaking gamepieces.
    fun pathAndIntake(path: PathPlannerPath): Command =
        grabGamepiece(
            path,
            noteDetector,
            drivetrain, shooter, groundIntake
        )




    val trajGroupName = "6pSpeakerBeta"
    val paths = PathPlannerPaths.fromChoreoTrajectoryGroup(trajGroupName)

    addRequirements(drivetrain, shooter, groundIntake)

    runOnce {
        drivetrain.poseEstimator.resetToChoreoTrajectory(trajGroupName)
    }

    // note 1
    +shooter.shootInSpeaker(0.7)

    // note 2
    +pathAndIntake(paths[0])
    +shooter.shootInSpeaker(0.7)

    // note 3
    +pathAndIntake(paths[1])
    runParallelUntilAllFinish{
        +AutoBuilder.followPath(paths[2])

        +shooter.setAngleCommand(PivotAngle.SPEAKER)
    }
    +shooter.shootInSpeaker(0.9)

    // note 4
    +pathAndIntake(paths[3])
    runParallelUntilAllFinish{
        +AutoBuilder.followPath(paths[4])

        +shooter.setAngleCommand(PivotAngle.SPEAKER)
    }
    +shooter.shootInSpeaker(0.7)

    // note 5
    +pathAndIntake(paths[4])
    +shooter.shootInSpeaker(0.8)

    +pathAndIntake(paths[5])
    +shooter.shootInSpeaker(0.9)

    runOnce{
        drivetrain.stop()
        shooter.setIdle()
        groundIntake.setIdle()
    }
}