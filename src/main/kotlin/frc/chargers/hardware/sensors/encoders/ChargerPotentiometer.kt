@file:Suppress("unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogPotentiometer

/**
 * A wrapper around WPILib's [AnalogPotentiometer] with units support.
 */
class ChargerPotentiometer(
    val channel: Int,
    fullRange: Angle,
    offset: Angle = 0.degrees,
    var inverted: Boolean = false
): PositionEncoder {
    val base = AnalogPotentiometer(AnalogInput(channel), fullRange.inUnit(degrees), offset.inUnit(degrees))

    override val angularPosition: Angle
        get() = (if (inverted) -1.0 else 1.0) * base.get().ofUnit(degrees)
}