package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain

fun basicTaxi(drivetrain: EncoderHolonomicDrivetrain): Command =
    buildCommand{
        loopFor(5.seconds, drivetrain){
            drivetrain.swerveDrive(-0.2,0.0,0.0, fieldRelative = false)
        }

        onEnd{
            drivetrain.stop()
        }
    }