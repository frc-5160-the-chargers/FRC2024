package frc.robot.commands

import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.robot.PATHFIND_CONSTRAINTS


private val ACCEPTABLE_DISTANCE_BEFORE_PATHFIND = 0.3.meters

/**
 * Follows a path,
 * pathfinding if the distance is large enough.
 */
fun followPathOptimal(
    drivetrain: EncoderHolonomicDrivetrain,
    path: PathPlannerPath
): Command = buildCommand(name = "FollowPathOptimal"){
    val pathStartPose = path.previewStartingHolonomicPose.ofUnit(meters)
    val pathEndPose = path.pathPoses.last().ofUnit(meters)

    val drivetrainPose by getOnceDuringRun {
        drivetrain.poseEstimator.robotPose
    }

    runIf(
        {
            drivetrainPose.distanceTo(pathStartPose.flipWhenNeeded()) > ACCEPTABLE_DISTANCE_BEFORE_PATHFIND &&
            drivetrainPose.distanceTo(pathEndPose.flipWhenNeeded()) > drivetrainPose.distanceTo(pathStartPose.flipWhenNeeded())
        },
        onTrue = AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS),
        onFalse = AutoBuilder.followPath(path)
    )
}