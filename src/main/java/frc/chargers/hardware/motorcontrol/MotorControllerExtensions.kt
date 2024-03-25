@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.motorcontrol.MotorController


/**
 * An extension property that acts as a replacement for MotorController.get()
 * and MotorController.set().
 */
public var MotorController.speed: Double
    get() = this.get()
    set(value) { this.set(value) }

/**
 * Sets the voltage of a [MotorController] using the kmeasure voltage unit.
 */
public fun MotorController.setVoltage(voltage: Voltage){
    setVoltage(voltage.inUnit(volts))
}



