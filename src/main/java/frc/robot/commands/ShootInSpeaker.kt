package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun shootInSpeaker(
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

    addRequirements(shooter, groundIntake, pivot)

    val spinupStartTime by getOnceDuringRun{ fpgaTimestamp() }

    if (movePivot){
        runParallelUntilFirstCommandFinishes{
            +pivot.setAngleCommand(PivotAngle.SPEAKER)

            loop{
                shooter.outtakeAtSpeakerSpeed()
            }
        }
    }

    loopUntil( { fpgaTimestamp() - spinupStartTime > shooterSpinUpTime } ){
        shooter.outtakeAtSpeakerSpeed()
    }

    if (shooter.hasNoteDetector){
        // sets time limit as 2.5 seconds
        runSequentiallyFor(2.5.seconds){
            // runs until note passes through the shooter once
            loopUntil({shooter.hasNote}){ runShooting() }
            loopUntil({!shooter.hasNote}){ runShooting() }
            // then, runs the shooter for a short period of time
            loopFor(0.5.seconds){ runShooting() }
        }
    }else{
        loopFor(1.5.seconds){ runShooting() }
    }

    onEnd{
        shooter.setIdle()
        groundIntake.setIdle()
    }
}