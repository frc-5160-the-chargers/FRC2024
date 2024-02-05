package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun runGroundIntake(
    shooter: Shooter,
    groundIntake: GroundIntake,
    indefinite: Boolean = false
): Command = buildCommand {
    fun setPower(){
        shooter.setSpeed(-0.2)
        groundIntake.spin(0.7)
    }

    addRequirements(shooter, groundIntake)

    +shooter.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

    if (indefinite){
        loop{
            setPower()
        }
    }else{
        loopUntil( { shooter.hasGamepiece } ){
            setPower()
        }
    }
}