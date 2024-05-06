@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.rotations
import edu.wpi.first.wpilibj.DigitalSource
import edu.wpi.first.wpilibj.DutyCycle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration

/**
 * Creates a [ChargerDutyCycleEncoder] with configuration.
 */
public inline fun ChargerDutyCycleEncoder(
    channel: Int,
    configure: DutyCycleEncoderConfiguration.() -> Unit
): ChargerDutyCycleEncoder = ChargerDutyCycleEncoder(channel).also{
    val configuration = DutyCycleEncoderConfiguration().apply(configure)
    it.configure(configuration)
}

/**
 * Creates a [ChargerDutyCycleEncoder] with configuration.
 */
public inline fun ChargerDutyCycleEncoder(
    source: DigitalSource,
    configure: DutyCycleEncoderConfiguration.() -> Unit
): ChargerDutyCycleEncoder = ChargerDutyCycleEncoder(source).also{
    val configuration = DutyCycleEncoderConfiguration().apply(configure)
    it.configure(configuration)
}

/**
 * Creates a [ChargerDutyCycleEncoder] with configuration.
 */
public inline fun ChargerDutyCycleEncoder(
    dutyCycle: DutyCycle,
    configure: DutyCycleEncoderConfiguration.() -> Unit
): ChargerDutyCycleEncoder = ChargerDutyCycleEncoder(dutyCycle).also{
    val configuration = DutyCycleEncoderConfiguration().apply(configure)
    it.configure(configuration)
}

/**
 * An Adapter of WPILib's [DutyCycleEncoder] class; consists of REV through bore encoders, CTRE mag encoders.
 */
public class ChargerDutyCycleEncoder: DutyCycleEncoder, PositionEncoder, HardwareConfigurable<DutyCycleEncoderConfiguration> {
    public constructor(channel: Int): super(channel)
    public constructor(source: DigitalSource): super(source)
    public constructor(dutyCycle: DutyCycle): super(dutyCycle)

    private var inverted: Boolean = false

    override val angularPosition: Angle
        get() = (if (inverted) -1.0 else 1.0) * absolutePosition.ofUnit(rotations)

    override fun configure(configuration: DutyCycleEncoderConfiguration) {
        configuration.connectedFrequencyThreshold?.let { setConnectedFrequencyThreshold(it) }
        configuration.dutyCycleRange?.let{
            setDutyCycleRange(it.start,it.endInclusive)
        }
        configuration.positionOffset?.let{
            positionOffset = it
        }

        configuration.inverted?.let{
            inverted = it
        }
    }
}

public data class DutyCycleEncoderConfiguration(
    var connectedFrequencyThreshold: Int? = null,
    var dutyCycleRange: ClosedRange<Double>? = null,
    var positionOffset: Double? = null,
    var inverted: Boolean? = null,
): HardwareConfiguration