@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.encoders.absolute

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.CANcoderConfiguration as CTRECANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder as CTRECANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.wpilibextensions.delay


/**
 * Creates a [ChargerCANcoder] with inline configuration.
 */
public inline fun ChargerCANcoder(
    deviceId: Int,
    canBus: String = "",
    factoryDefault: Boolean = true,
    configure: CANcoderConfiguration.() -> Unit
): ChargerCANcoder = ChargerCANcoder(deviceId,canBus,factoryDefault).also{
    it.configure(CANcoderConfiguration().apply(configure))
}

/**
 * A wrapper for the CTRE's CANcoder class, with integration into chargerlib.
 *
 * @see CTRECANcoder
 * @see CANcoderConfiguration
 */
public class ChargerCANcoder(
    deviceId: Int,
    canBus: String = "",
    factoryDefault: Boolean = true
): CTRECANcoder(deviceId, canBus), ResettableEncoder, HardwareConfigurable<CANcoderConfiguration> {

    init{
        if (factoryDefault){
            configurator.apply(CTRECANcoderConfiguration(),0.02)
            println("CANcoder has been factory defaulted.")
        }
    }

    /**
     * Represents the absolute encoder of the CANcoder.
     */
    public val absolute: ResettableEncoder = AbsoluteEncoderAdaptor()

    private inner class AbsoluteEncoderAdaptor: ResettableEncoder by this, HardwareConfigurable<CANcoderConfiguration> by this{
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



    private val allConfigErrors: LinkedHashSet<StatusCode> = linkedSetOf()
    private var configAppliedProperly = true
    private fun StatusCode.updateConfigStatus(): StatusCode {
        if (this != StatusCode.OK){
            if (RobotBase.isSimulation()){
                println("A Phoenix Device did not configure properly; however, this was ignored because the code is running in simulation.")
            }else{
                delay(200.milli.seconds)
                allConfigErrors.add(this)
                configAppliedProperly = false
            }
        }
        return this
    }
    
    override fun configure(configuration: CANcoderConfiguration){
        configAppliedProperly = true
        safeConfigure(
            deviceName = "ChargerCANcoder(id = $deviceID)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ) {
            allConfigErrors.clear()
            val baseConfig = CTRECANcoderConfiguration()
            configurator.refresh(baseConfig)
            applyChanges(baseConfig,configuration)
            configurator.apply(baseConfig,0.02).updateConfigStatus()


            configuration.positionUpdateFrequency?.let{
                position.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                absolutePosition.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
            }

            configuration.velocityUpdateFrequency?.let{
                velocity.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                unfilteredVelocity.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
            }

            configuration.filterVelocity?.let{ filterVelocity = it }
            return@safeConfigure configAppliedProperly
        }
    }
}


public data class CANcoderConfiguration(
    var futureProofConfigs: Boolean? = null,
    var sensorDirection: SensorDirectionValue? = null,
    var absoluteSensorRange: AbsoluteSensorRangeValue? = null,
    var magnetOffset: Angle? = null,
    var filterVelocity: Boolean? = null,

    var positionUpdateFrequency: Frequency? = null,
    var velocityUpdateFrequency: Frequency? = null
): HardwareConfiguration

internal fun applyChanges(ctreConfig: CTRECANcoderConfiguration, chargerConfig: CANcoderConfiguration): CTRECANcoderConfiguration{
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