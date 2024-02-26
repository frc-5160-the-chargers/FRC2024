package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.PATHFIND_CONSTRAINTS


private val ACCEPTABLE_DISTANCE_BEFORE_PATHFIND = 0.3.meters
private fun distanceBetween(poseA: UnitPose2d, poseB: UnitPose2d): Distance =
    abs((poseA - poseB).norm)

/**
 * Follows a path,
 * pathfinding if the distance is large enough.
 */
fun followPathOptimal(
    drivetrain: EncoderHolonomicDrivetrain,
    path: PathPlannerPath
): Command = buildCommand {
    val pathStartPose = path.previewStartingHolonomicPose.ofUnit(meters)
    val pathEndPose = path.pathPoses.last().ofUnit(meters)

    val drivetrainPose by getOnceDuringRun {
        drivetrain.poseEstimator.robotPose
    }

    runIf(
        {
            drivetrainPose.distanceTo(pathStartPose) > ACCEPTABLE_DISTANCE_BEFORE_PATHFIND &&
            drivetrainPose.distanceTo(pathEndPose) > distanceBetween(pathStartPose, pathStartPose)
        },
        onTrue = AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS),
        onFalse = AutoBuilder.followPath(path)
    )
}