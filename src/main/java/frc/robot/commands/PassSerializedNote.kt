package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.shooter.Shooter

val DEFAULT_PASS_TIME_WHEN_NO_SENSOR_PRESENT = 0.5.seconds

fun passSerializedNote(
    groundIntake: GroundIntakeSerializer,
    shooter: Shooter,
): Command = buildCommand {
    addRequirements(groundIntake, shooter)

    if (shooter.hasNoteDetector){
        loopUntil({shooter.hasNote}){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooter()
        }
    }else{
        loopFor(DEFAULT_PASS_TIME_WHEN_NO_SENSOR_PRESENT){
            println("No sensor No sensor!")
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooter()
        }
    }

    onEnd{
        groundIntake.setIdle()
        shooter.setIdle()
    }
}