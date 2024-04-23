@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.REVLibError
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.hardware.motorcontrol.rev.util.ChargerSparkConfiguration
import frc.chargers.hardware.motorcontrol.rev.util.SparkEncoderAdaptor
import frc.chargers.hardware.motorcontrol.rev.util.SparkEncoderType
import frc.chargers.hardware.motorcontrol.rev.util.SparkPIDHandler
import frc.chargers.utils.filterInvalid
import frc.chargers.utils.withSetter
import frc.chargers.wpilibextensions.delay

/**
 * A convenience function to create a [ChargerSparkMax],
 * which uses a function with the context of a [ChargerSparkConfiguration]
 * to configure the motor.
 *
 * Like the constructor, this function factory defaults the motor by default;
 * set factoryDefault = false to turn this off.
 *
 * ```
 * // example
 * val neo = ChargerSparkMax(5){ inverted = false }
 */
public inline fun ChargerSparkMax(
    deviceId: Int,
    type: MotorType = MotorType.kBrushless,
    factoryDefault: Boolean = true,
    configure: ChargerSparkConfiguration.() -> Unit
): ChargerSparkMax = ChargerSparkMax(
    deviceId, type, factoryDefault, ChargerSparkConfiguration().apply(configure)
)




/**
 * Represents a Spark Max motor controller.
 * Includes everything in the REV Robotics [CANSparkMax] class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * Creating an instance of this class factory will factory default the motor;
 * set factoryDefault = false to turn this off.
 *
 * @see com.revrobotics.CANSparkMax
 * @see ChargerSparkConfiguration
 */
public class ChargerSparkMax(
    deviceId: Int,
    type: MotorType = MotorType.kBrushless,
    factoryDefault: Boolean = true,
    configuration: ChargerSparkConfiguration? = null
) : CANSparkMax(deviceId, type), MotorizedComponent, HardwareConfigurable<ChargerSparkConfiguration>{
    private var encoderType: SparkEncoderType = SparkEncoderType.Regular()

    init{
        if (factoryDefault) {
            restoreFactoryDefaults()
            delay(200.milli.seconds)
            println("ChargerSparkMax has been factory defaulted.")
        }

        if (configuration != null){
            configure(configuration)
        }
    }


    /**
     * The encoder of the spark max.
     */
    override var encoder: SparkEncoderAdaptor = SparkEncoderAdaptor(this, encoderType)

    /**
     * Adds a generic amount of followers to the Spark Max, where all followers
     * mirror this motor's direction, regardless of invert.
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    override fun withFollowers(vararg followers: MotorizedComponent): MotorizedComponent {
        val nonRevFollowers = mutableListOf<MotorizedComponent>()
        for (follower in followers){
            if (follower is CANSparkBase){
                follower.follow(this)
            }else{
                nonRevFollowers.add(follower)
            }
        }
        return super.withFollowers(*nonRevFollowers.toTypedArray())
    }

    override val statorCurrent: Current by filterInvalid{ outputCurrent.ofUnit(amps) }

    override var hasInvert: Boolean
        get() = getInverted()
        set(value) = setInverted(value)

    override var appliedVoltage: Voltage
        by filterInvalid{ appliedOutput * busVoltage.ofUnit(volts) }
            .withSetter{ setVoltage(it.siValue) }




    /**
     * @see frc.chargers.hardware.motorcontrol.rev.util.SparkPIDHandler
     */
    private val pidHandler = SparkPIDHandler(motor = this, encoderAdaptor = encoder)

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




    private var allConfigErrors: MutableList<REVLibError> = mutableListOf()

    override fun configure(configuration: ChargerSparkConfiguration) {
        configuration.encoderType?.let{ configEncoderType ->
            encoderType = configEncoderType
            encoder = SparkEncoderAdaptor(this, configEncoderType)
        }

        // chargerlib defined function used for safe configuration.
        safeConfigure(
            deviceName = "ChargerSparkMax(id = $deviceId)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ){
            /**
             * Configures common configurations between motors that inherit [CANSparkBase].
             * Returns a List of [REVLibError]'s
             *
             * @see frc.chargers.hardware.motorcontrol.rev.util.ChargerSparkConfiguration
             */
            allConfigErrors = configuration.applyTo(this).toMutableList()

            return@safeConfigure allConfigErrors.any{ it != REVLibError.kOk }
        }

        if (RobotBase.isReal()) {
            delay(200.milli.seconds)
            burnFlash()
        }
    }

}
