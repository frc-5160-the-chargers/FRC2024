package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.NoteVisualizer
import frc.robot.hardware.subsystems.shooter.Shooter

fun shootInSpeaker(
    shooter: Shooter,
    groundIntake: GroundIntakeSerializer,
    pivot: Pivot,
    shooterSpinUpTime: Time = 0.3.seconds,
): Command = buildCommand {
    addRequirements(shooter, groundIntake, pivot)

    if (RobotBase.isSimulation()){
        +NoteVisualizer.shootInSpeakerCommand()
    }

    +pivot.setAngleCommand(PivotAngle.SPEAKER)

    loopFor(shooterSpinUpTime){
        shooter.outtakeAtSpeakerSpeed()
    }

    fun runShooting(){
        shooter.outtakeAtSpeakerSpeed()
        groundIntake.passToShooterFast()
    }

    if (shooter.hasNoteDetector){
        // only runs for max 3 seconds
        runSequentiallyFor(3.seconds){
            // runs until note passes through the shooter once
            loopUntil({shooter.hasNote}){ runShooting() }
            loopUntil({!shooter.hasNote}){ runShooting() }
            // then, runs the shooter for a short period of time
            loopFor(0.4.seconds){ runShooting() }
        }
    }else{
        loopFor(1.seconds){ runShooting() }
    }

    onEnd{
        shooter.setIdle()
        groundIntake.setIdle()
    }
}