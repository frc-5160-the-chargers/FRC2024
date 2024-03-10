package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter

val DEFAULT_PASS_TIME_WHEN_NO_SENSOR_PRESENT = 0.5.seconds

fun passSerializedNote(
    groundIntake: GroundIntakeSerializer,
    shooter: Shooter,
    pivot: Pivot
): Command = buildCommand {
    addRequirements(groundIntake, shooter, pivot)

    //+pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

    if (shooter.hasNoteDetector){
        loopUntil({shooter.hasNote}){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooter()
        }
    }else{
        loopFor(DEFAULT_PASS_TIME_WHEN_NO_SENSOR_PRESENT){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooter()
        }
    }

    +idleSubsystems(shooter = shooter, pivot = pivot, groundIntake = groundIntake)
}