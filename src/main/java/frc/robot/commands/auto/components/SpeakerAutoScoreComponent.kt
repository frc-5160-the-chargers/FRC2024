package frc.robot.commands.auto.components

import com.pathplanner.lib.path.PathPlannerPath

data class SpeakerAutoScoreComponent(
    val grabPath: PathPlannerPath,
    val scorePath: PathPlannerPath? = null,
    val shooterShouldStartDuringPath: Boolean = false,
){
    companion object{
        fun fromChoreo(
            grabPathName: String,
            scorePathName: String? = null,
            shooterShouldStartDuringPath: Boolean = false
        ): SpeakerAutoScoreComponent =
            SpeakerAutoScoreComponent(
                PathPlannerPath.fromChoreoTrajectory(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromChoreoTrajectory(scorePathName) else null,
                shooterShouldStartDuringPath,
            )

        fun fromPathPlanner(
            grabPathName: String,
            scorePathName: String? = null,
            shooterShouldStartDuringPath: Boolean = false
        ): SpeakerAutoScoreComponent =
            SpeakerAutoScoreComponent(
                PathPlannerPath.fromPathFile(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromPathFile(scorePathName) else null,
                shooterShouldStartDuringPath,
            )
    }
}