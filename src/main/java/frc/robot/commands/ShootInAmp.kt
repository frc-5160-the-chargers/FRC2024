package frc.robot.commands

import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

val quickAmpAngleMargin = LoggedDashboardNumber("QuickAmpAngleMarginDeg", 40.0)

fun shootInAmp(
    shooter: Shooter,
    pivot: Pivot,
    quickScore: Boolean = false
): Command = buildCommand("Shoot In Amp"){
    val outtakeCommand = buildCommand {
        if (shooter.hasNoteDetector){
            // outtake should not take more than 2 seconds
            runSequentiallyFor(2.seconds){
                loopUntil({shooter.hasNote}, shooter){
                    shooter.outtakeAtAmpSpeed()
                }

                loopWhile({shooter.hasNote}, shooter){
                    shooter.outtakeAtAmpSpeed()
                }

                loopFor(0.4.seconds, shooter){
                    shooter.outtakeAtAmpSpeed()
                }
            }
        }else{
            loopFor(0.7.seconds, shooter){
                shooter.outtakeAtAmpSpeed()
            }
        }
    }

    if (quickScore){
        runParallelUntilAllFinish{
            +pivot.setAngleCommand(PivotAngle.AMP)

            runSequentially{
                waitUntil{ abs(pivot.angle - PivotAngle.AMP) > quickAmpAngleMargin.get().ofUnit(degrees) }

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