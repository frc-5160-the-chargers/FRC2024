@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.*
import com.revrobotics.CANSparkLowLevel.*
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.motorcontrol.rev.util.*
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.delay

/**
 * Represents a type of spark max encoder.
 *
 * Options include [Regular], [Alternate] or [Absolute].
 */
public sealed class SparkMaxEncoderType{

    public data class Regular(
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkMaxEncoderType()

    public data class Alternate(
        val category: SparkMaxAlternateEncoder.Type,
        val countsPerRev: Int,
        val encoderMeasurementPeriod: Time? = null,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkMaxEncoderType()

    public data class Absolute(
        val category: SparkAbsoluteEncoder.Type,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkMaxEncoderType()
}




/**
 * A class that represents possible configurations for a spark max motor controller.
 *
 * @see ChargerSparkMax
 */
public class ChargerSparkMaxConfiguration(
    public var encoderType: SparkMaxEncoderType? = null,
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
 * A convenience function to create a [ChargerSparkMax],
 * which uses a function with the context of a [ChargerSparkMaxConfiguration]
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
    configure: ChargerSparkMaxConfiguration.() -> Unit
): ChargerSparkMax = ChargerSparkMax(
    deviceId, type, factoryDefault, ChargerSparkMaxConfiguration().apply(configure)
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
 * @see ChargerSparkMaxConfiguration
 */
public class ChargerSparkMax(
    deviceId: Int,
    type: MotorType = MotorType.kBrushless,
    factoryDefault: Boolean = true,
    configuration: ChargerSparkMaxConfiguration? = null
) : CANSparkMax(deviceId, type), SmartEncoderMotorController, HardwareConfigurable<ChargerSparkMaxConfiguration>{
    private val nonRevFollowers: MutableSet<SmartEncoderMotorController> = mutableSetOf()

    private var encoderType: SparkMaxEncoderType = SparkMaxEncoderType.Regular()

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
    override var encoder: SparkEncoderAdaptor = getEncoder(encoderType)
        private set

    /**
     * @see frc.chargers.hardware.motorcontrol.rev.util.SparkEncoderAdaptor
     */
    private fun getEncoder(encoderType: SparkMaxEncoderType): SparkEncoderAdaptor{
        this.encoderType = encoderType

        return when (encoderType){
            is SparkMaxEncoderType.Regular -> SparkEncoderAdaptor(
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

            is SparkMaxEncoderType.Alternate -> SparkEncoderAdaptor(
                super.getAlternateEncoder(
                    SparkMaxAlternateEncoder.Type.kQuadrature,
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

            is SparkMaxEncoderType.Absolute -> SparkEncoderAdaptor(
                super.getAbsoluteEncoder(encoderType.category).apply{
                    // property access syntax setters
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
    public fun withFollowers(vararg followers: SmartEncoderMotorController): ChargerSparkMax {
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
    public fun withInvertedFollowers(vararg followers: SmartEncoderMotorController): ChargerSparkMax {
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


    private var previousCurrent = Current(0.0)
    private var previousTemp = 0.0
    private var previousVoltage = Voltage(0.0)

    override val appliedCurrent: Current
        get() = outputCurrent.ofUnit(amps)
            .revertIfInvalid(previousCurrent)
            .also{ previousCurrent = it }

    override val tempCelsius: Double
        get() = motorTemperature
            .revertIfInvalid(previousTemp)
            .also{ previousTemp = it }

    override val appliedVoltage: Voltage
        get() = (get() * busVoltage.ofUnit(volts))
            .revertIfInvalid(previousVoltage)
            .also{ previousVoltage = it }



    /**
     * @see frc.chargers.hardware.motorcontrol.rev.util.SparkPIDHandler
     */
    private val pidHandler = SparkPIDHandler(motor = this, encoderAdaptor = encoder)

    override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean,
        extraVoltage: Voltage
    ): Unit = pidHandler.setAngularPosition(target, pidConstants, continuousWrap, extraVoltage, *nonRevFollowers.toTypedArray())

    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    ): Unit = pidHandler.setAngularVelocity(target, pidConstants, feedforward)




    private var allConfigErrors: MutableList<REVLibError> = mutableListOf()

    override fun configure(configuration: ChargerSparkMaxConfiguration) {
        configuration.encoderType?.let{ encoderType ->
            encoder = getEncoder(encoderType)
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
                            is SparkMaxEncoderType.Alternate -> {
                                status4 = FAST_PERIODIC_FRAME_STRATEGY
                            }

                            is SparkMaxEncoderType.Absolute -> {
                                if (MotorData.POSITION in frameConfig.utilizedData){
                                    status5 = FAST_PERIODIC_FRAME_STRATEGY
                                }
                                if (MotorData.VELOCITY in frameConfig.utilizedData){
                                    status6 = FAST_PERIODIC_FRAME_STRATEGY
                                }
                            }

                            is SparkMaxEncoderType.Regular -> {}
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
