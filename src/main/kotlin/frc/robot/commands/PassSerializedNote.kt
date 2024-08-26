package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.subsystems.GroundIntakeSerializer
import frc.robot.subsystems.NoteObserver
import frc.robot.subsystems.shooter.Shooter

fun passSerializedNote(
    noteObserver: NoteObserver,
    groundIntake: GroundIntakeSerializer,
    shooter: Shooter,
): Command = buildCommand("Pass Serialized Note") {
    require(groundIntake, shooter)

    if (RobotBase.isReal()){
        loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }

        loopFor(0.4.seconds){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }

    }else{
        loopFor(0.9.seconds){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }
    }

    onEnd{
        groundIntake.setIdle()
        shooter.setIdle()
    }
}