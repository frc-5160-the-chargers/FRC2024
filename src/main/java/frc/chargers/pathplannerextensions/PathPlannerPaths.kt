@file:Suppress("unused")
package frc.chargers.pathplannerextensions

import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.Filesystem
import java.io.File

/**
 * A utility class for loading multiple path files as a list of paths.
 */
object PathPlannerPaths{
    fun fromPathFiles(vararg fileNames: String): List<PathPlannerPath> =
        fileNames.map{ name -> PathPlannerPath.fromPathFile(name) }


    fun fromChoreoTrajectoryGroup(name: String): List<PathPlannerPath>{
        val files = File(Filesystem.getDeployDirectory(), "choreo")
            .listFiles { file -> file.name.toRegex().matches("$name\\.\\d+\\.traj") }
            ?: error("It seems that you do not have a trajectory group with the name $name.")

        return files.map{ file -> PathPlannerPath.fromChoreoTrajectory(file.name) }
    }
}