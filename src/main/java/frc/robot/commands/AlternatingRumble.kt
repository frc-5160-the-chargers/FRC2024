package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.robot.hardware.inputdevices.DriverController

fun alternatingRumble(): Command = buildCommand{
    runOnce{
        DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
    }

    waitFor(0.2.seconds)

    runOnce{
        DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }

    waitFor(0.2.seconds)

    onEnd {
        DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }
}.repeatedly()