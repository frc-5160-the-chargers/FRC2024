@file:Suppress("RedundantVisibilityModifier", "unused") 
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
public class ChargerPotentiometer(
    input: AnalogInput,
    fullRange: Angle,
    offset: Angle = 0.degrees,
    private val inverted: Boolean = false
): AnalogPotentiometer(input, fullRange.inUnit(degrees), offset.inUnit(degrees)), PositionEncoder {
    public constructor(
        channel: Int,
        fullRange: Angle,
        offset: Angle = 0.degrees,
        inverted: Boolean = false
    ) : this(AnalogInput(channel), fullRange, offset, inverted)

    override val angularPosition: Angle
        get() = (if (inverted) -1.0 else 1.0) * get().ofUnit(degrees)
}