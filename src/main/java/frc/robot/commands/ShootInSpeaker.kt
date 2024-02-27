package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
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

    power: Double = 0.8,
    shooterSpinUpTime: Time = 0.3.seconds,
    timeout: Time? = 0.7.seconds
): Command = buildCommand {
    addRequirements(shooter, groundIntake, pivot)

    +pivot.setAngleCommand(PivotAngle.SPEAKER)

    runParallelUntilAllFinish{
        if (timeout != null){
            loopFor(timeout){
                shooter.outtake(power)
            }
        }else{
            loop{
                shooter.outtake(power)
            }
        }

        runSequentially{
            waitFor(shooterSpinUpTime)

            if (timeout != null){
                loopFor(timeout - shooterSpinUpTime){
                    groundIntake.setConveyorVoltage(10.volts)
                }
            }else{
                loop{
                    groundIntake.setConveyorVoltage(10.volts)
                }
            }
        }
    }

    onEnd{
        shooter.setIdle()
        groundIntake.setIdle()
    }
}