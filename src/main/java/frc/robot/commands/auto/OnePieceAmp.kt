package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun onePieceAmp(
    shooter: Shooter,
    revertShooterPositionOnEnd: Boolean = false
): Command = buildCommand{
    +shooter.setAngleCommand(PivotAngle.AMP)

    loopFor(0.5.seconds, shooter){
        shooter.setSpeed(0.5)
    }

    if (revertShooterPositionOnEnd){
        +shooter.setAngleCommand(PivotAngle.IDLE)
    }
}