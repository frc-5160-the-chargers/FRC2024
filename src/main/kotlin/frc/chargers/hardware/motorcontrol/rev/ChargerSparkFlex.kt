@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import edu.wpi.first.wpilibj.RobotBase
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.hardware.configuration.ConfigurableHardware
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.rev.util.ChargerSparkConfiguration
import frc.chargers.hardware.motorcontrol.rev.util.SparkEncoderAdaptor
import frc.chargers.hardware.motorcontrol.rev.util.SparkEncoderType
import frc.chargers.hardware.motorcontrol.rev.util.SparkPIDHandler
import frc.chargers.utils.filterInvalid
import frc.chargers.utils.withSetter
import frc.chargers.wpilibextensions.delay

/**
 * A convenience function to create a [ChargerSparkFlex],
 * which uses a function with the context of a [ChargerSparkConfiguration]
 * to configure the motor.
 *
 * Like the constructor, this function factory defaults the motor by default;
 * set factoryDefault = false to turn this off.
 *
 * ```
 * // example
 * val neo = ChargerSparkFlex(5){ inverted = false }
 */
public inline fun ChargerSparkFlex(
    deviceId: Int,
    factoryDefault: Boolean = true,
    // context function; configure is called as if it was a function within the ChargerSparkFlexConfiguration class itself
    configure: ChargerSparkConfiguration.() -> Unit
): ChargerSparkFlex = ChargerSparkFlex(
    deviceId, factoryDefault,
    ChargerSparkConfiguration().apply(configure)
)

/**
 * A wrapper around REV's [CANSparkFlex], with support for Kmeasure units
 * and integration with the rest of the library.
 *
 * Creating an instance of this class factory will factory default the motor;
 * set factoryDefault = false to turn this off.
 *
 * @see ChargerSparkConfiguration
 * @see com.revrobotics.CANSparkFlex
 */
public class ChargerSparkFlex(
    deviceId: Int,
    factoryDefault: Boolean = true,
    configuration: ChargerSparkConfiguration? = null
) : CANSparkFlex(deviceId, MotorType.kBrushless), Motor, ConfigurableHardware<ChargerSparkConfiguration> {
    private var encoderType: SparkEncoderType = SparkEncoderType.Regular()

    init{
        if (factoryDefault) {
            restoreFactoryDefaults()
            delay(200.milli.seconds)
            println("ChargerSparkFlex has been factory defaulted.")
        }
        if (configuration != null){
            configure(configuration)
        }
    }

    /**
     * The encoder of the spark flex.
     */
    override var encoder: SparkEncoderAdaptor = SparkEncoderAdaptor(this, encoderType)
        private set

    /**
     * The current applied current of the spark flex.
     */
    override val statorCurrent: Current by filterInvalid { outputCurrent.ofUnit(amps) }

    override var hasInvert: Boolean
        get() = getInverted()
        set(value) = setInverted(value)

    /**
     * A getter-setter variable that fetches the applied voltage to the spark flex,
     * as well as the requested voltage.
     *
     * Note: [withSetter] is an extension function that gives ReadOnlyProperty's setters
     */
    override var appliedVoltage: Voltage by
        filterInvalid{ appliedOutput * busVoltage.ofUnit(volts) }
            .withSetter{ setVoltage(it.siValue) }

    override fun setBrakeMode(shouldBrake: Boolean){
        idleMode = if (shouldBrake) IdleMode.kBrake else IdleMode.kCoast
    }

    /**
     * Adds a generic amount of followers to the Spark Max, where all followers
     * mirror this motor's direction, regardless of invert.
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    override fun withFollowers(vararg followers: Motor): Motor {
        val nonRevFollowers = mutableListOf<Motor>()
        for (follower in followers){
            if (follower is CANSparkBase){
                follower.follow(this)
            }else{
                nonRevFollowers.add(follower)
            }
        }
        return super.withFollowers(*nonRevFollowers.toTypedArray())
    }

    /**
     * @see frc.chargers.hardware.motorcontrol.rev.util.SparkPIDHandler
     */
    private val pidHandler = SparkPIDHandler(this, encoderAdaptor = encoder)

    override fun setPositionSetpoint(
        rawPosition: Angle,
        pidConstants: PIDConstants,
        continuousInput: Boolean,
        feedforward: Voltage
    ): Unit = pidHandler.setAngularPosition(rawPosition, pidConstants, continuousInput, feedforward)

    override fun setVelocitySetpoint(
        rawVelocity: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    ): Unit = pidHandler.setAngularVelocity(rawVelocity, pidConstants, feedforward)

    override fun configure(configuration: ChargerSparkConfiguration) {
        configuration.encoderType?.let{ configEncoderType ->
            encoder = SparkEncoderAdaptor(this, configEncoderType)
            encoderType = configEncoderType
        }

        configuration.applyTo(this)

        if (RobotBase.isReal()) {
            delay(200.milli.seconds)
            burnFlash()
        }
    }
}