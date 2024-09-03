package frc.robot.rigatoni.commands

import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.distanceTo
import frc.chargers.wpilibextensions.flipWhenRed
import frc.robot.rigatoni.PATHFIND_CONSTRAINTS

private val ACCEPTABLE_DISTANCE_BEFORE_PATHFIND = 0.3.meters

/**
 * Follows a path,
 * pathfinding if the distance is large enough.
 */
fun followPathOptimal(
    drivetrain: EncoderHolonomicDrivetrain,
    path: PathPlannerPath
): Command = buildCommand(name = "FollowPathOptimal"){
    val pathStartPose = path.previewStartingHolonomicPose
    val pathEndPose = path.pathPoses.last()

    runIf(
        {
            val drivetrainPose = drivetrain.robotPose
            drivetrainPose.distanceTo(pathStartPose.flipWhenRed()) > ACCEPTABLE_DISTANCE_BEFORE_PATHFIND &&
                drivetrainPose.distanceTo(pathEndPose.flipWhenRed()) > drivetrainPose.distanceTo(pathStartPose.flipWhenRed())
        },
        onTrue = AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS),
        onFalse = AutoBuilder.followPath(path)
    )
}