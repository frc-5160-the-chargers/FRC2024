package frc.robot.commands.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.pathplannerextensions.PathPlannerPaths
import frc.robot.commands.enableAimToSpeaker
import frc.robot.commands.grabGamepiece
import frc.robot.commands.shootInSpeaker
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter


@Suppress("unused")
fun fivePieceSpeakerBeta(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntake,

    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline
): Command = buildCommand {
    // adds a command that makes the drivebase follow a path, while intaking gamepieces.
    fun pathAndIntake(path: PathPlannerPath): Command =
        grabGamepiece(
            path,
            noteDetector,
            drivetrain, pivot, shooter, groundIntake
        )




    val trajGroupName = "5pAutoLeft"
    val paths = PathPlannerPaths.fromChoreoTrajectoryGroup(trajGroupName)

    addRequirements(drivetrain, shooter, groundIntake, pivot)

    runOnce {
        drivetrain.poseEstimator.resetToChoreoTrajectory(trajGroupName)
    }

    // note 1
    +shootInSpeaker(shooter, pivot, 0.7)

    // note 2
    +pathAndIntake(paths[0])
    +shootInSpeaker(shooter, pivot, 0.9)

    // note 3
    +pathAndIntake(paths[1])
    +enableAimToSpeaker(drivetrain, apriltagVision)
    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(paths[2], PATHFIND_CONSTRAINTS)

        +pivot.setAngleCommand(PivotAngle.SPEAKER)
    }
    +shootInSpeaker(shooter, pivot, 0.7)

    // note 4
    +pathAndIntake(paths[3])
    +enableAimToSpeaker(drivetrain, apriltagVision)
    +shootInSpeaker(shooter, pivot, 0.7)

    // note 5
    +pathAndIntake(paths[4])
    +enableAimToSpeaker(drivetrain, apriltagVision)
    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(paths[2], PATHFIND_CONSTRAINTS)

        +pivot.setAngleCommand(PivotAngle.SPEAKER)
    }
    +shootInSpeaker(shooter, pivot, 0.7)

    runOnce{
        drivetrain.stop()
        shooter.setIdle()
        groundIntake.setIdle()
    }

    +basicTaxi(drivetrain)
}