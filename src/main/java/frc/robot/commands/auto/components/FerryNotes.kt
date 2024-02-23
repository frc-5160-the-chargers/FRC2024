package frc.robot.commands.auto.components

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.driveToNoteAndIntake
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot

private val OUTTAKE_TIME = 0.3.seconds

/*
Runs a choreo auto that ferries notes into the alliance side.
 */
fun ferryNotes(
    noteDetector: ObjectVisionPipeline?,
    drivetrain: EncoderHolonomicDrivetrain,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
): Command = buildCommand(name = "Note Ferry(2 notes)"){
    var pathCounter = 1
    repeat(2){
        val grabPath = PathPlannerPath.fromChoreoTrajectory("FerryPath.$pathCounter")

        if (noteDetector != null){
            +driveToNoteAndIntake(
                noteDetector,
                drivetrain, pivot, groundIntake,
                path = grabPath,
            )
        }else{
            runParallelUntilFirstCommandFinishes{
                +AutoBuilder.followPath(grabPath)

                loop{
                    groundIntake.intake(pivot)
                }
            }
        }

        +AutoBuilder.followPath(
            PathPlannerPath.fromChoreoTrajectory("FerryPath." + (pathCounter + 1))
        )

        loopFor(OUTTAKE_TIME){
            groundIntake.outtake()
        }

        runOnce{
            groundIntake.setIdle()
        }

        pathCounter += 2
    }
}