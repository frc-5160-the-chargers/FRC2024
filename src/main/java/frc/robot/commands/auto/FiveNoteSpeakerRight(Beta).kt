package frc.robot.commands.auto

import kotlin.math.abs
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.pathplannerextensions.PathPlannerPaths
import frc.robot.PATHFIND_CONSTRAINTS
import frc.robot.commands.*
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter


@Suppress("unused")
fun fiveNoteSpeakerRight(
    aprilTagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
): Command = buildCommand {
    // adds a command that makes the drivebase follow a path, while intaking gamepieces.
    fun pathAndIntake(path: PathPlannerPath): Command =
        driveToNoteAndIntake(
            noteDetector, drivetrain,
            shooter, pivot, groundIntake, path
        )

    fun aimToSpeakerIfNecessary(
        path: PathPlannerPath,
        shouldDelay: Boolean = false
    ): Command = ConditionalCommand(
        if (shouldDelay){
            enableAimToSpeaker(drivetrain, aprilTagVision).andThen(WaitCommand(0.2))
        }else{
            enableAimToSpeaker(drivetrain, aprilTagVision)
        },
        InstantCommand{}
    ){
        // condition
        abs(path.allPathPoints.last().rotationTarget.target.degrees - drivetrain.heading.inUnit(degrees)) < 10.0
    }


    val trajGroupName = "5pAutoLeft"
    val paths = PathPlannerPaths.fromChoreoTrajectoryGroup(trajGroupName)

    addRequirements(drivetrain, shooter, groundIntake, pivot)

    runOnce {
        drivetrain.poseEstimator.resetToChoreoTrajectory(trajGroupName)
    }

    // note 1
    +shootInSpeaker(shooter, groundIntake, pivot, 0.7)

    // note 2
    +pathAndIntake(paths[0])
    +shootInSpeaker(shooter, groundIntake, pivot, 0.9)

    // note 3
    +pathAndIntake(paths[1])
    +aimToSpeakerIfNecessary(paths[1], shouldDelay = false)
    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(paths[2], PATHFIND_CONSTRAINTS)

        +pivot.setAngleCommand(PivotAngle.SPEAKER)
    }
    +shootInSpeaker(shooter, groundIntake, pivot, 0.7)

    // note 4
    +pathAndIntake(paths[3])
    +aimToSpeakerIfNecessary(paths[1], shouldDelay = true)
    +shootInSpeaker(shooter, groundIntake, pivot, 0.7)

    // note 5
    +pathAndIntake(paths[4])
    +aimToSpeakerIfNecessary(paths[1], shouldDelay = false)
    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(paths[2], PATHFIND_CONSTRAINTS)

        +pivot.setAngleCommand(PivotAngle.SPEAKER)
    }
    +shootInSpeaker(shooter, groundIntake, pivot, 0.7)

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    +basicTaxi(drivetrain)
}
