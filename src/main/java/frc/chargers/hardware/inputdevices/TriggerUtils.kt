@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.external.frc6328.DoublePressTracker

public fun Trigger.withDoubleClick(clickTimeout: Time? = null): Trigger =
    if (clickTimeout == null){
        DoublePressTracker.createTrigger(this)
    }else{
        DoublePressTracker.createTrigger(this, clickTimeout.inUnit(seconds))
    }

public fun Trigger.onDoubleClick(clickTimeout: Time? = null, command: Command){
    withDoubleClick(clickTimeout).onTrue(command)
}

public fun Trigger.whileTrueContinuous(command: Command) {
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

