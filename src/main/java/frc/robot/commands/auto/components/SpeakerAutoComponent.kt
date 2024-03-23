package frc.robot.commands.auto.components

import com.pathplanner.lib.path.PathPlannerPath

data class SpeakerAutoComponent(
    val grabPath: PathPlannerPath,
    val scorePath: PathPlannerPath? = null,
){
    companion object{
        fun fromChoreo(
            grabPathName: String,
            scorePathName: String? = null
        ): SpeakerAutoComponent =
            SpeakerAutoComponent(
                PathPlannerPath.fromChoreoTrajectory(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromChoreoTrajectory(scorePathName) else null,
            )

        fun fromPathPlanner(
            grabPathName: String,
            scorePathName: String? = null
        ): SpeakerAutoComponent =
            SpeakerAutoComponent(
                PathPlannerPath.fromPathFile(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromPathFile(scorePathName) else null,
            )
    }
}