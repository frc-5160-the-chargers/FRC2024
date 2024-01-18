package frc.robot.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.pathplannerextensions.GoalEndState
import frc.chargers.pathplannerextensions.PathConstraints
import frc.chargers.pathplannerextensions.bezierFromPoses
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d

private val maxDistanceBeforePathfindCalled = UnitTranslation2d(1.meters, 1.meters)

// uses custom PathConstraints fake overload(function) for kmeasure quantities
private val pathfindingConstraints = PathConstraints(
    Velocity(2.0),
    Acceleration(2.0),
    AngularVelocity(2.0),
    AngularAcceleration(2.0)
)

private fun shouldPathfind(targetPose: UnitPose2d, currentPose: UnitPose2d): Boolean{
    // flipWhenNeeded is an extension function defined in chargerlib
    val translation = (currentPose - targetPose).translation
    // uses absolute value overloads for kmeasure quantities
    return abs(translation.x) > maxDistanceBeforePathfindCalled.x &&
            abs(translation.y) > maxDistanceBeforePathfindCalled.y
}

/**
 * Executes AD star pathfinding only if the distance is large enough;
 * uses on the fly paths otherwise.
 */
fun EncoderHolonomicDrivetrain.pathfindOptimal(
    blueAlliancePose: UnitPose2d,
    goalEndVelocity: Velocity = Velocity(0.0),
    rotationDelayDistance: Distance = Distance(0.0)
): Command =
    if (shouldPathfind(blueAlliancePose.flipWhenNeeded(), this.poseEstimator.robotPose)){
        AutoBuilder.pathfindToPose(
            blueAlliancePose.flipWhenNeeded().inUnit(meters),
            pathfindingConstraints,
            goalEndVelocity.siValue, rotationDelayDistance.siValue
        )
    }else{
        AutoBuilder.followPath(
            PathPlannerPath(
                bezierFromPoses(blueAlliancePose).map{it.siValue},
                pathfindingConstraints,
                // custom overload of GoalEndState
                GoalEndState(goalEndVelocity, Angle(0.0))
            )
        )
    }