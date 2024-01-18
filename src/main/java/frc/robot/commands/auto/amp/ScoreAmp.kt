package frc.robot.commands.auto.amp

import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun Shooter.scoreAmpCommand(): Command = buildCommand {
    addRequirements(this@scoreAmpCommand) // requires shooter

    loopUntil( {hasHitPivotTarget} ){
        setPivotPosition(PivotAngle.AMP)
    }

    loopFor(1.5.seconds){
        spin(5.volts)
    }
}