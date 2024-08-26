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
    init {
        if (factoryDefault){
            applyConfiguration(configuration ?: ChargerCANcoderConfiguration(), factoryDefault = true)
        }else{
            if (configuration != null){
                applyConfiguration(configuration, factoryDefault = false)
            }
        }
    }

    private val posSignal = position
    private var velSignal = velocity

    /**
     * Represents the absolute encoder of the CANcoder.
     */
    public val absolute: Encoder = AbsoluteEncoderAdaptor()
    private inner class AbsoluteEncoderAdaptor: Encoder by this, ConfigurableHardware<ChargerCANcoderConfiguration> by this {
        private val absolutePosSignal = absolutePosition

        override val angularPosition: Angle
            get() = absolutePosSignal.refresh(true).value.ofUnit(rotations)
    }

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

    override fun configure(configuration: ChargerCANcoderConfiguration) =
        applyConfiguration(configuration, factoryDefault = false)

    private fun applyConfiguration(configuration: ChargerCANcoderConfiguration, factoryDefault: Boolean) {
        val ctreConfig = CANcoderConfiguration()
        if (!factoryDefault){
            // if you don't want to factory default,
            // refresh the ctre configuration with the device's existing configs
            // before making changes
            configurator.refresh(ctreConfig)
        }
        ctreConfig.apply{
            configuration.futureProofConfigs?.let{
                FutureProofConfigs = it
            }

            MagnetSensor.apply{
                configuration.sensorDirection?.let{SensorDirection = it}
                configuration.absoluteSensorRange?.let{AbsoluteSensorRange = it}
                configuration.magnetOffset?.let{MagnetOffset = it.inUnit(rotations)}
            }
        }
        configurator.apply(ctreConfig, 0.02)

        configuration.positionUpdateFrequency?.let{
            position.setUpdateFrequency(it.inUnit(hertz))
            absolutePosition.setUpdateFrequency(it.inUnit(hertz))
        }
        configuration.velocityUpdateFrequency?.let{
            velocity.setUpdateFrequency(it.inUnit(hertz))
            unfilteredVelocity.setUpdateFrequency(it.inUnit(hertz))
        }
        if (configuration.filterVelocity == true){
            velSignal = velocity
        }else if (configuration.filterVelocity == false){
            velSignal = unfilteredVelocity
        }
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