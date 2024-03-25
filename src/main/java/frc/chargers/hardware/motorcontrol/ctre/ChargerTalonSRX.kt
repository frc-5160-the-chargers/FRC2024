@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import kotlin.math.roundToInt

private const val TALON_SRX_ENCODER_UNITS_PER_ROTATION = 2048 // From https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
private const val TIMEOUT_MILLIS = 1000

public typealias PIDIndex = Int
public typealias SlotIndex = Int
public typealias CustomParameterIndex = Int
public typealias CustomParameterValue = Int


/**
 * Represents a TalonSRX powering a redline/ CIM motor.
 *
 * This function supports inline configuration using the "[configure]" lambda function,
 * which has the context of a [ChargerTalonSRXConfiguration].
 *
 * You do not need to manually factory default this motor, as it is factory defaulted on startup,
 * before configuration. This setting can be changed by setting factoryDefault = false.
 *
 * ```
 * // example
 * val motor = redlineSRX(canId = 6){ inverted = false }
 */
public inline fun redlineSRX(
    deviceNumber: Int,
    encoderTicksPerRotation: Int = 1024,
    factoryDefault: Boolean = true,
    configure: ChargerTalonSRXConfiguration.() -> Unit = {}
): ChargerTalonSRX =
    ChargerTalonSRX(deviceNumber,encoderTicksPerRotation).apply{
        if (factoryDefault) {
            configFactoryDefault()
            println("TalonSRX has been factory defaulted.")
        }
        val config = ChargerTalonSRXConfiguration().apply(configure)
        configure(config)
    }

public class TalonSRXEncoderAdapter(
    private val ctreMotorController: IMotorController,
    private val pidIndex: Int,
    private val anglePerPulse: Angle
) : ResettableEncoder, IMotorController by ctreMotorController {
    public constructor(
        ctreMotorController: IMotorController,
        pidIndex: Int,
        pulsesPerRotation: Int /* Can't use Double here or both constructors will have the same JVM signature */
    ) : this(ctreMotorController, pidIndex, (1/pulsesPerRotation.toDouble()).ofUnit(rotations))

    override fun setZero(newZero: Angle) {
        val errorCode = ctreMotorController.setSelectedSensorPosition((newZero/anglePerPulse).siValue, pidIndex, DEFAULT_TIMEOUT_MS)
        if (errorCode != ErrorCode.OK){
            error("When setting the zero of a talon srx's encoder, an error ocurred: $errorCode")
        }
    }

    override val angularPosition: Angle
        get() = ctreMotorController.getSelectedSensorPosition(pidIndex) * anglePerPulse

    override val angularVelocity: AngularVelocity
        get() = ctreMotorController.getSelectedSensorVelocity(pidIndex) * anglePerPulse / timeBetweenPulses

    public companion object {
        private const val DEFAULT_TIMEOUT_MS = 500
        private val timeBetweenPulses = 100.milli.seconds
    }
}




/**
 * Creates an instance of a [ChargerTalonSRX] through a "[configure]" lambda function,
 * which has the context of a [ChargerTalonSRXConfiguration].
 *
 * ```
 * // example
 * val motor = ChargerTalonSRX(canId = 6){ feedbackRemoteSensorId = 5 }
 */
public inline fun ChargerTalonSRX(
    deviceNumber: Int,
    encoderTicksPerRotation: Int,
    factoryDefault: Boolean = true,
    configure: ChargerTalonSRXConfiguration.() -> Unit
): ChargerTalonSRX = ChargerTalonSRX(
    deviceNumber, encoderTicksPerRotation, factoryDefault,
    ChargerTalonSRXConfiguration().apply(configure)
)




/**
 * Represents a TalonSRX motor controller.
 * Includes everything in the CTRE TalonSRX class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 *
 * Creating an instance of this class factory will factory default the motor;
 * set factoryDefault = false to turn this off.
 *
 * In addition, the ChargerTalonSRX still uses phoenix v5,
 * as phoenix v6 scraps support for the TalonSRX.
 *
 * @see com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
 * @see ChargerTalonSRXConfiguration
 */
public class ChargerTalonSRX(
    deviceNumber: Int,
    private val encoderTicksPerRotation: Int,
    factoryDefault: Boolean = true,
    configuration: ChargerTalonSRXConfiguration? = null
) : WPI_TalonSRX(deviceNumber), EncoderMotorController, HardwareConfigurable<ChargerTalonSRXConfiguration>{

    private val nonSRXFollowers: MutableSet<EncoderMotorController> = mutableSetOf()

    init{
        if (factoryDefault){
            configFactoryDefault()
        }

        if (configuration != null){
            configure(configuration)
        }
    }


    override val encoder: Encoder
        get() = TalonSRXEncoderAdapter(
            ctreMotorController = this,
            pidIndex = 0,
            pulsesPerRotation = encoderTicksPerRotation
        )


    override fun configure(configuration: ChargerTalonSRXConfiguration) {
        configuration.inverted?.let(::setInverted)
        configuration.expiration?.let { expiration = it.inUnit(seconds) }
        configuration.safetyEnabled?.let(::setSafetyEnabled)

        val encoderStep = (1.0 / TALON_SRX_ENCODER_UNITS_PER_ROTATION).ofUnit(rotations)

        configuration.openLoopRampTimeFromNeutralToFull?.let { configOpenloopRamp(it.inUnit(seconds), TIMEOUT_MILLIS) }
        configuration.closedLoopRampTimeFromNeutralToFull?.let { configClosedloopRamp(it.inUnit(seconds), TIMEOUT_MILLIS) }
        configuration.peakOutputForwardPercent?.let { configPeakOutputForward(it, TIMEOUT_MILLIS) }
        configuration.peakOutputReversePercent?.let { configPeakOutputReverse(it, TIMEOUT_MILLIS) }
        configuration.nominalOutputForwardPercent?.let { configNominalOutputForward(it, TIMEOUT_MILLIS) }
        configuration.nominalOutputReversePercent?.let { configNominalOutputReverse(it, TIMEOUT_MILLIS) }
        configuration.neutralDeadbandPercent?.let { configNeutralDeadband(it, TIMEOUT_MILLIS) }
        configuration.voltageCompensationSaturationVoltage?.let {
            configVoltageCompSaturation(it.inUnit(volts), TIMEOUT_MILLIS)
        }
        configuration.voltageMeasurementFilterSamples?.let {
            configVoltageMeasurementFilter(it, TIMEOUT_MILLIS)
        }
        configuration.voltageCompensationEnabled?.let(::enableVoltageCompensation)
        configuration.selectedFeedbackSensors.forEach { (pidIndex, feedbackDevice) ->
            configSelectedFeedbackSensor(feedbackDevice, pidIndex, TIMEOUT_MILLIS)
        }
        configuration.selectedFeedbackCoefficients.forEach { (pidIndex, coefficient) ->
            configSelectedFeedbackCoefficient(coefficient, pidIndex, TIMEOUT_MILLIS)
        }
        configuration.sensorTermFeedbackDevices.forEach { (sensorTerm, feedbackDevice) ->
            configSensorTerm(sensorTerm, feedbackDevice, TIMEOUT_MILLIS)
        }
        configuration.controlFramePeriods.forEach { (controlFrame, period) ->
            setControlFramePeriod(controlFrame, period.inUnit(milli.seconds).roundToInt())
        }
        configuration.statusFramePeriods.forEach { (statusFrame, period) ->
            setStatusFramePeriod(statusFrame, period.inUnit(milli.seconds).roundToInt(), TIMEOUT_MILLIS)
        }

        configuration.forwardLimitSwitchSource?.let {  (type, normalOpenOrClose, deviceId) ->
            configForwardLimitSwitchSource(type, normalOpenOrClose, deviceId, TIMEOUT_MILLIS)
        }
        configuration.reverseLimitSwitchSource?.let {  (type, normalOpenOrClose, deviceId) ->
            configReverseLimitSwitchSource(type, normalOpenOrClose, deviceId, TIMEOUT_MILLIS)
        }
        configuration.forwardSoftLimitThreshold?.let { configForwardSoftLimitThreshold(it.inUnit(encoderStep),
            TIMEOUT_MILLIS
        ) }
        configuration.reverseSoftLimitThreshold?.let { configReverseSoftLimitThreshold(it.inUnit(encoderStep),
            TIMEOUT_MILLIS
        ) }
        configuration.forwardSoftLimitEnable?.let { configForwardSoftLimitEnable(it, TIMEOUT_MILLIS) }
        configuration.reverseSoftLimitEnable?.let { configReverseSoftLimitEnable(it, TIMEOUT_MILLIS) }


        configuration.customParameters.forEach { (i, customParameter) ->
            configSetCustomParam(customParameter, i, TIMEOUT_MILLIS)
        }

        println("ChargerTalonSRX(id = $deviceID) has been configured.")
    }
}

/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonSRX.
 *
 * @see ChargerTalonSRX
 */
public data class ChargerTalonSRXConfiguration(
    var inverted: Boolean? = null,
    var invertSensorPhase: Boolean? = null,
    var expiration: Time? = null,
    var safetyEnabled: Boolean? = null,
    var openLoopRampTimeFromNeutralToFull: Time? = null,
    var closedLoopRampTimeFromNeutralToFull: Time? = null,
    var peakOutputForwardPercent: Double? = null,
    var peakOutputReversePercent: Double? = null,
    var nominalOutputForwardPercent: Double? = null,
    var nominalOutputReversePercent: Double? = null,
    var neutralDeadbandPercent: Double? = null,
    var voltageCompensationSaturationVoltage: Voltage? = null,
    var voltageMeasurementFilterSamples: Int? = null,
    var voltageCompensationEnabled: Boolean? = null,
    val selectedFeedbackSensors: MutableMap<PIDIndex, FeedbackDevice> = mutableMapOf(),
    val selectedFeedbackCoefficients: MutableMap<PIDIndex, Double> = mutableMapOf(),
    val sensorTermFeedbackDevices: MutableMap<SensorTerm, FeedbackDevice> = mutableMapOf(),
    val controlFramePeriods: MutableMap<ControlFrame, Time> = mutableMapOf(),
    val statusFramePeriods: MutableMap<StatusFrame, Time> = mutableMapOf(),

    public var forwardLimitSwitchSource: LimitSwitchConfig? = null,
    public var reverseLimitSwitchSource: LimitSwitchConfig? = null,
    public var forwardSoftLimitThreshold: Angle? = null,
    public var reverseSoftLimitThreshold: Angle? = null,
    public var forwardSoftLimitEnable: Boolean? = null,
    public var reverseSoftLimitEnable: Boolean? = null,

    val customParameters: MutableMap<CustomParameterIndex, CustomParameterValue> = mutableMapOf()
): HardwareConfiguration


public data class LimitSwitchConfig(val type: RemoteLimitSwitchSource, val normalOpenOrClose: LimitSwitchNormal, val deviceId: Int)
