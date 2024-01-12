@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.pathplannerextensions


import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d


/**
 * Creates a bezier from many [UnitPose2d]s.
 */
public fun bezierFromPoses(vararg poses: UnitPose2d): List<UnitTranslation2d> =
    PathPlannerPath.bezierFromPoses(poses.map{it.siValue}).map{UnitTranslation2d(it)}


public fun PathPlannerPath(
    bezierPoints: List<UnitTranslation2d>,
    constraints: PathConstraints,
    endState: GoalEndState,
): PathPlannerPath = PathPlannerPath(
    bezierPoints.map{it.siValue},
    constraints,endState
)

