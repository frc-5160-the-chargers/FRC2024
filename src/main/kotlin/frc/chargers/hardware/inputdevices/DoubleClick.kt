@file:Suppress("unused")
package frc.chargers.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.external.frc6328.DoublePressTracker

fun Trigger.withDoubleClick(clickTimeout: Time? = null): Trigger =
    if (clickTimeout == null){
        DoublePressTracker.createTrigger(this)
    }else{
        DoublePressTracker.createTrigger(this, clickTimeout.inUnit(seconds))
    }

fun Trigger.onDoubleClick(command: Command){
    withDoubleClick().onTrue(command)
}

fun Trigger.onDoubleClick(clickTimeout: Time, command: Command){
    withDoubleClick(clickTimeout).onTrue(command)
}

fun Trigger.onClickAndHold(command: Command){
    withDoubleClick().whileTrue(command)
}

fun Trigger.onClickAndHold(clickTimeout: Time, command: Command){
    withDoubleClick(clickTimeout).whileTrue(command)
}

