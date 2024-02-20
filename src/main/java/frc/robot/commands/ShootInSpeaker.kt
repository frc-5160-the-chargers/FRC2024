package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun shootInSpeaker(
    shooter: Shooter,
    groundIntake: GroundIntakeSerializer,
    pivot: Pivot,

    power: Double = 0.8
): Command = buildCommand {
    +pivot.setAngleCommand(PivotAngle.SPEAKER)

    runParallelUntilAllFinish{
        loopFor(0.7.seconds, shooter){
            shooter.outtake(power)
        }

        runSequentially{
            waitFor(0.4.seconds)

            loopFor(0.3.seconds, groundIntake){
                groundIntake.setConveyorVoltage(10.volts)
            }
        }
    }

    runOnce(shooter, groundIntake){
        shooter.setIdle()
        groundIntake.setIdle()
    }
}