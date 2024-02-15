package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun shootInAmp(
    pivot: Pivot,
    shooter: Shooter,
    percentOut: Double,
    time: Time = 0.3.seconds,
    stowOnEnd: Boolean = false
): Command = buildCommand {
    +pivot.setAngleCommand(PivotAngle.AMP)

    loopFor(time, shooter) {
        shooter.outtake(percentOut)
    }

    runOnce(shooter){
        shooter.setIdle()
    }

    if (stowOnEnd) {
        +pivot.setAngleCommand(PivotAngle.STOWED)
    }
}