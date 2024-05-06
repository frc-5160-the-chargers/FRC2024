@file:Suppress("unused")
package frc.chargers.hardware.inputdevices

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger

fun Trigger.whileTrueContinuous(command: Command) {
    var pressedLast = asBoolean
    CommandScheduler.getInstance().defaultButtonLoop
        .bind{
            val pressed = asBoolean
            if (pressed) {
                command.schedule()
            } else if (pressedLast) {
                command.cancel()
            }
            pressedLast = pressed
        }
}
