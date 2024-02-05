@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.revrobotics.*
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.rev.util.*
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.delay


/**
 * Represents a type of spark flex encoder.
 *
 * Options include [Regular], [External] or [Absolute].
 */
public sealed class SparkFlexEncoderType{
    /**
     * Represents a regular Spark Flex encoder.
     */
    public data class Regular(
        val optimizeStatusFrames: Boolean = true,
        val category: SparkRelativeEncoder.Type = SparkRelativeEncoder.Type.kHallSensor,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkFlexEncoderType()

    /**
     * Represents a Spark Flex Alternate encoder.
     */
    public data class External(
        val countsPerRev: Int,
        val category: SparkFlexExternalEncoder.Type = SparkFlexExternalEncoder.Type.kQuadrature,
        val encoderMeasurementPeriod: Time? = null,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkFlexEncoderType()

    /**
     * Represents an absolute encoder connected to a Spark Flex.
     */
    public data class Absolute(
        val category: SparkAbsoluteEncoder.Type,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkFlexEncoderType()
}


/**
 * A class that represents possible configurations for a spark flex motor controller.
 *
 * @see ChargerSparkFlex
 */
public class ChargerSparkFlexConfiguration(
    public var encoderType: SparkFlexEncoderType? = null,
    idleMode: CANSparkBase.IdleMode? = null,
    inverted: Boolean? = null,
    voltageCompensationNominalVoltage: Voltage? = null,
    canTimeout: Time? = null,
    closedLoopRampRate: Double? = null,
    openLoopRampRate: Double? = null,
    controlFramePeriod: Time? = null,
    periodicFrameConfig: PeriodicFrameConfig? = null,
    smartCurrentLimit: SmartCurrentLimit? = null,
    secondaryCurrentLimit: SecondaryCurrentLimit? = null,
    softLimits: MutableMap<CANSparkBase.SoftLimitDirection, Angle> = mutableMapOf(),
): SparkConfigurationBase(
    idleMode, inverted, voltageCompensationNominalVoltage, canTimeout, closedLoopRampRate, openLoopRampRate,
    controlFramePeriod, periodicFrameConfig, smartCurrentLimit, secondaryCurrentLimit, softLimits
)




/**
 * A convenience function to create a [ChargerSparkFlex],
 * which uses a function with the context of a [ChargerSparkFlexConfiguration]
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
    configure: ChargerSparkFlexConfiguration.() -> Unit
): ChargerSparkFlex = ChargerSparkFlex(
    deviceId, factoryDefault,
    ChargerSparkFlexConfiguration().apply(configure)
)




/**
 * A wrapper around REV's [CANSparkFlex], with support for Kmeasure units
 * and integration with the rest of the library.
 *
 * Creating an instance of this class factory will factory default the motor;
 * set factoryDefault = false to turn this off.
 *
 * @see ChargerSparkFlexConfiguration
 * @see com.revrobotics.CANSparkFlex
 */
public class ChargerSparkFlex(
    deviceId: Int,
    factoryDefault: Boolean = true,
    configuration: ChargerSparkFlexConfiguration? = null
) : CANSparkFlex(deviceId, MotorType.kBrushless), SmartEncoderMotorController, HardwareConfigurable<ChargerSparkFlexConfiguration> {
    private val nonRevFollowers: MutableSet<SmartEncoderMotorController> = mutableSetOf()

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

    private var encoderType: SparkFlexEncoderType = SparkFlexEncoderType.Regular()

    /**
     * The encoder of the spark flex.
     */
    override var encoder: SparkEncoderAdaptor = getEncoder(encoderType)
        private set

    /**
     * @see frc.chargers.hardware.motorcontrol.rev.util.SparkEncoderAdaptor
     */
    private fun getEncoder(encoderType: SparkFlexEncoderType): SparkEncoderAdaptor{
        this.encoderType = encoderType

        return when (encoderType){
            is SparkFlexEncoderType.Regular -> SparkEncoderAdaptor(
                super.getEncoder().apply{
                    // property access syntax setters
                    if (encoderType.averageDepth != null){
                        averageDepth = encoderType.averageDepth
                    }
                    if (encoderType.inverted != null){
                        inverted = encoderType.inverted
                    }
                }
            )

            is SparkFlexEncoderType.External -> SparkEncoderAdaptor(
                super.getExternalEncoder(
                    encoderType.category,
                    encoderType.countsPerRev
                ).apply{
                    // property access syntax setters
                    if (encoderType.encoderMeasurementPeriod != null){
                        measurementPeriod = encoderType.encoderMeasurementPeriod.inUnit(milli.seconds).toInt()
                    }
                    if (encoderType.averageDepth != null){
                        averageDepth = encoderType.averageDepth
                    }
                    if (encoderType.inverted != null){
                        inverted = encoderType.inverted
                    }
                }
            )

            is SparkFlexEncoderType.Absolute -> SparkEncoderAdaptor(
                super.getAbsoluteEncoder(encoderType.category).apply{
                    // property access syntax setters(replace setAverageDepth and getAverageDepth)
                    if (encoderType.averageDepth != null){
                        averageDepth = encoderType.averageDepth
                    }
                    if (encoderType.inverted != null){
                        inverted = encoderType.inverted
                    }
                }
            )
        }
    }


    private var previousCurrent = Current(0.0)
    private var previousTemp = 0.0
    private var previousVoltage = Voltage(0.0)

    override val appliedCurrent: Current
        get() = outputCurrent.ofUnit(amps)
            .revertIfInvalid(previousCurrent) // reverts value if it is NaN or infinity. Applies to Kmeasure Quantities and Doubles.
            .also{ previousCurrent = it }

    override val tempCelsius: Double
        get() = motorTemperature
            .revertIfInvalid(previousTemp)
            .also{ previousTemp = it }

    override val appliedVoltage: Voltage
        get() = (appliedOutput * busVoltage.ofUnit(volts))
            .revertIfInvalid(previousVoltage)
            .also{ previousVoltage = it }


    /**
     * Adds a generic amount of followers to the Spark Max, where all followers
     * mirror this motor's direction, regardless of invert.
     *
     * To make followers oppose the master's direction, see [withInvertedFollowers].
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    public fun withFollowers(vararg followers: SmartEncoderMotorController): ChargerSparkFlex{
        /**
         * @see frc.chargers.hardware.motorcontrol.rev.util.addFollowers
         */
        addFollowers(
            this,
            nonRevFollowerSetReference = nonRevFollowers,
            invert = false,
            *followers
        )
        return this
    }


    /**
     * Adds a generic amount of followers to the Spark Max, where all followers
     * run in the opposite direction of this motor, regardless of invert.
     *
     * To make followers oppose the master's direction, see [withFollowers].
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    public fun withInvertedFollowers(vararg followers: SmartEncoderMotorController): ChargerSparkFlex{
        /**
         * @see frc.chargers.hardware.motorcontrol.rev.util.addFollowers
         */
        addFollowers(
            this,
            nonRevFollowerSetReference = nonRevFollowers,
            invert = true,
            *followers
        )
        return this
    }





    override fun set(speed: Double){
        super.set(speed.coerceIn(-1.0..1.0))
        nonRevFollowers.forEach{ it.set(speed.coerceIn(-1.0..1.0)) }
    }


    override fun disable(){
        super.disable()
        nonRevFollowers.forEach{ it.disable() }
    }


    /**
     * @see frc.chargers.hardware.motorcontrol.rev.util.SparkPIDHandler
     */
    private val pidHandler = SparkPIDHandler(this, encoderAdaptor = encoder)

    override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean,
        extraVoltage: Voltage
    ): Unit = pidHandler.setAngularPosition(target, pidConstants, continuousWrap, extraVoltage)

    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    ): Unit = pidHandler.setAngularVelocity(target, pidConstants, feedforward)


    private var allConfigErrors: MutableList<REVLibError> = mutableListOf()

    override fun configure(configuration: ChargerSparkFlexConfiguration) {
        configuration.encoderType?.let{ encoderType ->
            encoder = getEncoder(encoderType)
        }
        // chargerlib defined function used for safe configuration.
        safeConfigure(
            deviceName = "ChargerSparkFlex(id = $deviceId)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ){
            /**
             * Configures common configurations between motors that inherit [CANSparkBase].
             * Returns a List of [REVLibError]'s
             *
             * @see frc.chargers.hardware.motorcontrol.rev.util.SparkConfigurationBase
             */
            allConfigErrors = configuration.applyTo(this).toMutableList()

            fun REVLibError.addError(){
                allConfigErrors.add(this)
            }

            when (val frameConfig = configuration.periodicFrameConfig){
                is PeriodicFrameConfig.Custom -> {
                    frameConfig.frames.forEach{ (frame, period) ->
                        setPeriodicFramePeriod(frame, period).addError()
                    }
                }

                is PeriodicFrameConfig.Optimized -> {
                    // status 0 is ignored due to it only being applicable to follower motors
                    var status1 = SLOW_PERIODIC_FRAME_STRATEGY
                    var status2 = SLOW_PERIODIC_FRAME_STRATEGY
                    // status 3 is skipped due to chargerlib not interfacing w/ analog encoders
                    var status4 = DISABLED_PERIODIC_FRAME_STRATEGY
                    var status5 = DISABLED_PERIODIC_FRAME_STRATEGY
                    var status6 = DISABLED_PERIODIC_FRAME_STRATEGY

                    if (MotorData.TEMPERATURE in frameConfig.utilizedData ||
                        MotorData.VELOCITY in frameConfig.utilizedData ||
                        MotorData.VOLTAGE in frameConfig.utilizedData
                    ) {
                        status1 = FAST_PERIODIC_FRAME_STRATEGY
                    }

                    if (MotorData.POSITION in frameConfig.utilizedData) {
                        status2 = FAST_PERIODIC_FRAME_STRATEGY
                    }

                    if (frameConfig.optimizeEncoderFrames){
                        when (this.encoderType){
                            is SparkFlexEncoderType.Absolute -> {
                                status4 = FAST_PERIODIC_FRAME_STRATEGY
                            }

                            is SparkFlexEncoderType.External -> {
                                if (MotorData.POSITION in frameConfig.utilizedData){
                                    status5 = FAST_PERIODIC_FRAME_STRATEGY
                                }
                                if (MotorData.VELOCITY in frameConfig.utilizedData){
                                    status6 = FAST_PERIODIC_FRAME_STRATEGY
                                }
                            }

                            is SparkFlexEncoderType.Regular -> {}
                        }
                        setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1).addError()
                        setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2).addError()
                        setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4).addError()
                        setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5).addError()
                        setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6).addError()
                    }
                }

                null -> {}
            }

            return@safeConfigure allConfigErrors.isEmpty()
        }

        if (RobotBase.isReal()) {
            delay(200.milli.seconds)
            burnFlash()
        }
    }


}