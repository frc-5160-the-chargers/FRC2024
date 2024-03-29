package frc.robot.commands.auto.components

import com.batterystaple.kmeasure.quantities.Time
import com.pathplanner.lib.path.PathPlannerPath


/**
 * Represents an additional step of an amp autonomous command.
 * This step includes a path that grabs a note, as well as a path that scores a note.
 *
 * This component can either direct an autonomous command to score an additional note,
 * or ferry a note.
 */
data class AmpAutoComponent(
    val grabPath: PathPlannerPath,
    val scorePath: PathPlannerPath,
    val type: Type = Type.SCORE_NOTE,
    val groundIntakePreSpinupTime: Time? = null,
){
    enum class Type{
        SCORE_NOTE, FERRY_NOTE
    }

    companion object{
        fun fromChoreo(
            grabPathName: String,
            scorePathName: String,
            type: Type = Type.SCORE_NOTE,
            groundIntakePreSpinupTime: Time? = null,
        ): AmpAutoComponent =
            AmpAutoComponent(
                PathPlannerPath.fromChoreoTrajectory(grabPathName),
                PathPlannerPath.fromChoreoTrajectory(scorePathName),
                type,
                groundIntakePreSpinupTime
            )

        fun fromPathPlanner(
            grabPathName: String,
            scorePathName: String,
            type: Type = Type.SCORE_NOTE,
            groundIntakePreSpinupTime: Time? = null,
        ): AmpAutoComponent =
            AmpAutoComponent(
                PathPlannerPath.fromPathFile(grabPathName),
                PathPlannerPath.fromPathFile(scorePathName),
                type,
                groundIntakePreSpinupTime
            )
    }
}