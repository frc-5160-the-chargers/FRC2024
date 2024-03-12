package frc.robot.commands.auto.components

import com.pathplanner.lib.path.PathPlannerPath

data class SpeakerAutoScoreComponent(
    val grabPath: PathPlannerPath,
    val scorePath: PathPlannerPath? = null,
    val shooterShouldStartDuringPath: Boolean = false,
    val shouldShootOnEnd: Boolean = true,
){
    companion object{
        fun fromChoreo(
            grabPathName: String,
            scorePathName: String? = null,
            shooterShouldStartDuringPath: Boolean = false,
            shouldShootOnEnd: Boolean = true
        ): SpeakerAutoScoreComponent =
            SpeakerAutoScoreComponent(
                PathPlannerPath.fromChoreoTrajectory(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromChoreoTrajectory(scorePathName) else null,
                shooterShouldStartDuringPath,
                shouldShootOnEnd
            )

        fun fromPathPlanner(
            grabPathName: String,
            scorePathName: String? = null,
            shooterShouldStartDuringPath: Boolean = false,
            shouldShootOnEnd: Boolean = true
        ): SpeakerAutoScoreComponent =
            SpeakerAutoScoreComponent(
                PathPlannerPath.fromPathFile(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromPathFile(scorePathName) else null,
                shooterShouldStartDuringPath,
                shouldShootOnEnd
            )
    }
}