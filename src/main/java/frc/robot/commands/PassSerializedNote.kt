package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.shooter.Shooter

fun passSerializedNote(
    groundIntake: GroundIntakeSerializer,
    shooter: Shooter,
): Command = buildCommand("Pass Serialized Note") {
    addRequirements(groundIntake, shooter)

    if (shooter.hasNoteDetector){
        loopUntil({shooter.hasNote}){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }

        loopFor(0.3.seconds){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }

    }else{
        loopFor(0.5.seconds){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }
    }

    onEnd{
        groundIntake.setIdle()
        shooter.setIdle()
    }
}