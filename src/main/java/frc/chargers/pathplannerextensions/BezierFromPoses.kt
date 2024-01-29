@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.pathplannerextensions


import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Translation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d


/**
 * Creates a bezier from many [UnitPose2d]s.
 */
public fun bezierFromPoses(vararg poses: UnitPose2d): List<Translation2d> =
    PathPlannerPath.bezierFromPoses( poses.map{it.siValue} )

