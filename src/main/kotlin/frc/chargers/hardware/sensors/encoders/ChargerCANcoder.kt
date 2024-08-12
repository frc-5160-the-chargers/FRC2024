@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.chargers.hardware.configuration.ConfigurableHardware
import frc.chargers.hardware.configuration.HardwareConfiguration


/**
 * Creates a [ChargerCANcoder] with inline configuration.
 */
public inline fun ChargerCANcoder(
    deviceId: Int,
    canBus: String = "",
    factoryDefault: Boolean = true,
    configure: ChargerCANcoderConfiguration.() -> Unit
): ChargerCANcoder = ChargerCANcoder(
    deviceId, canBus, factoryDefault,
    ChargerCANcoderConfiguration().apply(configure)
)

/**
 * A wrapper for the CTRE's CANcoder class, with integration into chargerlib.
 *
 * @see CANcoder
 * @see ChargerCANcoderConfiguration
 */
public class ChargerCANcoder(
    deviceId: Int,
    canBus: String = "",
    factoryDefault: Boolean = true,
    configuration: ChargerCANcoderConfiguration? = null
): CANcoder(deviceId, canBus), ResettableEncoder, ConfigurableHardware<ChargerCANcoderConfiguration> {
    init{
        val baseConfig = CANcoderConfiguration()

        if (!factoryDefault){
            configurator.refresh(baseConfig)
        }else{
            // an empty CTRE CANcoderConfiguration will factory default the encoder if applied.
            println("CANcoder will factory default.")
        }

        if (configuration != null){
            configure(configuration, baseConfig)
        }else {
            configure(ChargerCANcoderConfiguration(), baseConfig)
        }
    }

    /**
     * Represents the absolute encoder of the CANcoder.
     */
    public val absolute: ResettableEncoder = AbsoluteEncoderAdaptor()

    private inner class AbsoluteEncoderAdaptor: ResettableEncoder by this, ConfigurableHardware<ChargerCANcoderConfiguration> by this{
        private val absolutePosSignal = absolutePosition

        override val angularPosition: Angle
            get() = absolutePosSignal.refresh(true).value.ofUnit(rotations)
    }


    private var filterVelocity: Boolean = true
    private val posSignal = position
    private val velSignal = if (filterVelocity) velocity else unfilteredVelocity

    override fun setZero(newZero: Angle){
        setPosition(newZero.inUnit(rotations))
    }

    /**
     * Obtains the relative position from the CANcoder.
     */
    override val angularPosition: Angle
        get() = posSignal.refresh(true).value.ofUnit(rotations)

    /**
     * Obtains the velocity of the CANcoder.
     */
    override val angularVelocity: AngularVelocity
        get() = velSignal.refresh(true).value.ofUnit(rotations/seconds)



    override fun configure(configuration: ChargerCANcoderConfiguration) {
        // for a CTRE CANcoderConfiguration,
        // calling configurator.apply(configuration) will cause all configurations not explicitly specified
        // to revert to the factory default.
        // in contrast, ChargerCANcoderConfiguration has all values default to null,
        // where "null" is an untouched configuration(aka preserved from previous configurations).
        // thus, to acheive this functionality, a CTRE configuration must be refreshed FIRST
        // before changes are applied and the configuration is configured.
        val baseConfig = CANcoderConfiguration()
        configurator.refresh(baseConfig)
        configure(configuration, baseConfig)
    }
    
    public fun configure(configuration: ChargerCANcoderConfiguration, baseCANcoderConfiguration: CANcoderConfiguration){
        applyChanges(baseCANcoderConfiguration,configuration)
        configurator.apply(baseCANcoderConfiguration,0.02)


        configuration.positionUpdateFrequency?.let{
            position.setUpdateFrequency(it.inUnit(hertz))
            absolutePosition.setUpdateFrequency(it.inUnit(hertz))
        }

        configuration.velocityUpdateFrequency?.let{
            velocity.setUpdateFrequency(it.inUnit(hertz))
            unfilteredVelocity.setUpdateFrequency(it.inUnit(hertz))
        }

        configuration.filterVelocity?.let{ filterVelocity = it }
    }
}


public data class ChargerCANcoderConfiguration(
    var futureProofConfigs: Boolean? = null,
    var sensorDirection: SensorDirectionValue? = null,
    var absoluteSensorRange: AbsoluteSensorRangeValue? = null,
    var magnetOffset: Angle? = null,
    var filterVelocity: Boolean? = null,

    var positionUpdateFrequency: Frequency? = null,
    var velocityUpdateFrequency: Frequency? = null
): HardwareConfiguration

internal fun applyChanges(ctreConfig: CANcoderConfiguration, chargerConfig: ChargerCANcoderConfiguration): CANcoderConfiguration{
    ctreConfig.apply{
        chargerConfig.futureProofConfigs?.let{
            FutureProofConfigs = it
        }

        MagnetSensor.apply{
            chargerConfig.sensorDirection?.let{SensorDirection = it}
            chargerConfig.absoluteSensorRange?.let{AbsoluteSensorRange = it}
            chargerConfig.magnetOffset?.let{MagnetOffset = it.inUnit(rotations)}
        }
    }
    return ctreConfig
}