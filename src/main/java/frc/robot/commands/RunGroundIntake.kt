package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter


/**
 * Runs the ground intake and moves the pivot to the appropriate position,
 * to prevent serialized notes from coming out of the serializer.
 */
fun runGroundIntake(
    groundIntake: GroundIntakeSerializer,
    pivot: Pivot,
    shooter: Shooter,

    timeout: Time? = null
): Command = buildCommand{
    addRequirements(groundIntake, pivot, shooter)

    +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

    if (timeout != null){
        loopFor(timeout){
            shooter.intake(-0.08) // slight speaker outtake voltage to prevent note from going into it
            groundIntake.intake()
        }
    }else{
        loop{
            shooter.intake(-0.08) // slight speaker outtake voltage to prevent note from going into it
            groundIntake.intake()
        }
    }

    onEnd{
        shooter.setIdle()
        groundIntake.setIdle()
        pivot.setIdle()
    }
}