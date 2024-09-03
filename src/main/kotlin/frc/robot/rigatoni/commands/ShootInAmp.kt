package frc.robot.rigatoni.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.rigatoni.subsystems.NoteObserver
import frc.robot.rigatoni.subsystems.pivot.Pivot
import frc.robot.rigatoni.subsystems.pivot.PivotAngle
import frc.robot.rigatoni.subsystems.shooter.Shooter

fun shootInAmp(
    noteObserver: NoteObserver,
    shooter: Shooter,
    pivot: Pivot
): Command = buildCommand {
    require(noteObserver, shooter, pivot)

    +pivot.setAngleCommand(PivotAngle.AMP)

    if (RobotBase.isReal()){
        // outtake should not take more than 2 seconds
        runSequentiallyFor(2.seconds){
            loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){
                shooter.outtakeAtAmpSpeed()
            }

            loopUntil({noteObserver.state == NoteObserver.State.NoNote}){
                shooter.outtakeAtAmpSpeed()
            }

            loopFor(0.3.seconds){
                shooter.outtakeAtAmpSpeed()
            }
        }
    }else{
        loopFor(0.7.seconds){
            shooter.outtakeAtAmpSpeed()
        }
    }
}