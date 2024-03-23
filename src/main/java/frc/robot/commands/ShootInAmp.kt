package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun shootInAmp(
    shooter: Shooter,
    pivot: Pivot
): Command = buildCommand("Shoot In Amp"){
    +pivot.setAngleCommand(PivotAngle.AMP)

    if (shooter.hasNoteDetector){
        loopUntil({shooter.hasNote}, shooter){
            shooter.outtakeAtAmpSpeed()
        }

        loopWhile({shooter.hasNote}, shooter){
            shooter.outtakeAtAmpSpeed()
        }

        loopFor(0.5.seconds, shooter){
            shooter.outtakeAtAmpSpeed()
        }
    }else{
        loopFor(0.9.seconds, shooter){
            shooter.outtakeAtAmpSpeed()
        }
    }

    onEnd{
        pivot.setIdle()
        shooter.setIdle()
    }
}