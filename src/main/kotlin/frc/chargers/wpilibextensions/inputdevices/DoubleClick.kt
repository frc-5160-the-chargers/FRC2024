@file:Suppress("unused")
package frc.chargers.wpilibextensions.inputdevices

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc6328.DoublePressTracker

fun Trigger.withDoubleClick(clickTimeout: Time? = null): Trigger =
    if (clickTimeout == null){
        DoublePressTracker.createTrigger(this)
    }else{
        DoublePressTracker.createTrigger(this, clickTimeout.inUnit(seconds))
    }

fun Trigger.onDoubleClick(clickTimeout: Time? = null, command: Command){
    withDoubleClick(clickTimeout).onTrue(command)
}

fun Trigger.onClickAndHold(clickTimeout: Time? = null, command: Command){
    withDoubleClick(clickTimeout).whileTrue(command)
}

