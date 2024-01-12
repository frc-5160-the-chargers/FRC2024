@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogPotentiometer
import frc.chargers.hardware.sensors.encoders.PositionEncoder

/**
 * A wrapper around WPILib's [AnalogPotentiometer] with units support.
 */
public class ChargerPotentiometer(input: AnalogInput, fullRange: Angle, offset: Angle = 0.degrees) :
    AnalogPotentiometer(input, fullRange.inUnit(degrees), offset.inUnit(degrees)), PositionEncoder {
    public constructor(channel: Int, fullRange: Angle, offset: Angle = 0.degrees) : this(AnalogInput(channel), fullRange, offset)

    override val angularPosition: Angle
        get() = get().ofUnit(degrees)
}