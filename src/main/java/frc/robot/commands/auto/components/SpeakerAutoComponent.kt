package frc.robot.commands.auto.components

import com.pathplanner.lib.path.PathPlannerPath

data class SpeakerAutoComponent(
    val grabPath: PathPlannerPath,
    val scorePath: PathPlannerPath? = null,
    val spinupShooterDuringGrabPath: Boolean = true,
){
    companion object{
        fun fromChoreo(
            grabPathName: String,
            scorePathName: String? = null,
            spinupShooterDuringGrabPath: Boolean = true,
        ): SpeakerAutoComponent =
            SpeakerAutoComponent(
                PathPlannerPath.fromChoreoTrajectory(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromChoreoTrajectory(scorePathName) else null,
                spinupShooterDuringGrabPath
            )

        fun fromPathPlanner(
            grabPathName: String,
            scorePathName: String? = null,
            spinupShooterDuringGrabPath: Boolean = true,
        ): SpeakerAutoComponent =
            SpeakerAutoComponent(
                PathPlannerPath.fromPathFile(grabPathName),
                if (scorePathName != null) PathPlannerPath.fromPathFile(scorePathName) else null,
                spinupShooterDuringGrabPath
            )
    }
}