package frc.robot.rigatoni.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.robot.rigatoni.subsystems.GroundIntakeSerializer
import frc.robot.rigatoni.subsystems.NoteObserver
import frc.robot.rigatoni.subsystems.pivot.Pivot
import frc.robot.rigatoni.subsystems.pivot.PivotAngle
import frc.robot.rigatoni.subsystems.shooter.Shooter

fun shootInSpeaker(
    noteObserver: NoteObserver,
    shooter: Shooter,
    groundIntake: GroundIntakeSerializer,
    pivot: Pivot,
    shooterSpinUpTime: Time = 1.seconds,
    movePivot: Boolean = true
): Command = buildCommand("Shoot In Speaker"){
    fun runShooting(){
        shooter.outtakeAtSpeakerSpeed()
        groundIntake.passToShooterFast()
    }

    require(shooter, groundIntake, pivot)

    val spinupStartTime by getOnceDuringRun{ fpgaTimestamp() }

    if (movePivot){
        runParallelUntilFirstCommandFinishes{
            +pivot.setAngleCommand(PivotAngle.SPEAKER)

            loop{ shooter.outtakeAtSpeakerSpeed() }
        }
    }

    loopUntil( { fpgaTimestamp() - spinupStartTime > shooterSpinUpTime } ){
        shooter.outtakeAtSpeakerSpeed()
    }

    if (RobotBase.isReal()){
        // sets time limit as 2.5 seconds
        runSequentiallyFor(2.5.seconds){
            // runs until note passes through the shooter once
            loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){
                runShooting()
            }
            loopUntil({noteObserver.state == NoteObserver.State.NoNote}){
                runShooting()
            }
            // then, runs the shooter for a short period of time
            loopFor(0.4.seconds){
                runShooting()
            }
        }
    }else{
        loopFor(1.5.seconds){ runShooting() }
    }

    onEnd{
        shooter.setIdle()
        groundIntake.setIdle()
    }
}