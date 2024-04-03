package frc.robot.commands

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

val quickAmpDelay = LoggedDashboardNumber("QuickAmpDelay", 0.0)

fun shootInAmp(
    shooter: Shooter,
    pivot: Pivot,
    quickScore: Boolean = false
): Command = buildCommand("Shoot In Amp"){
    val outtakeCommand = buildCommand {
        if (shooter.hasNoteDetector){
            loopUntil({shooter.hasNote}, shooter){
                shooter.outtakeAtAmpSpeed()
            }

            loopWhile({shooter.hasNote}, shooter){
                shooter.outtakeAtAmpSpeed()
            }

            loopFor(0.4.seconds, shooter){
                shooter.outtakeAtAmpSpeed()
            }
        }else{
            loopFor(0.9.seconds, shooter){
                shooter.outtakeAtAmpSpeed()
            }
        }
    }

    if (quickScore){
        runParallelUntilAllFinish{
            +pivot.setAngleCommand(PivotAngle.AMP)

            runSequentially{
                val startTime by getOnceDuringRun{ fpgaTimestamp() }
                
                waitUntil{ fpgaTimestamp() - startTime > quickAmpDelay.get().ofUnit(seconds) }

                +outtakeCommand
            }
        }
    }else{
        +pivot.setAngleCommand(PivotAngle.AMP)

        +outtakeCommand
    }

    onEnd{
        pivot.setIdle()
        shooter.setIdle()
    }
}