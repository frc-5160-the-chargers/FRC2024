@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.math.inputModulus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*

/**
 * An adaptor to the encoder of a [TalonFX].
 */
public class TalonFXEncoderAdapter(
    private val motorController: TalonFX
): ResettableEncoder {
    private val positionSignal = motorController.position
    private val velocitySignal = motorController.velocity

    override fun setZero(newZero: Angle) {
        val errorCode = motorController.setPosition(newZero.inUnit(rotations))
        if (errorCode != StatusCode.OK){
            error("When attempting to zero a talon fx, an error occurred: $errorCode")
        }
    }

    override val angularPosition: Angle
        get() = positionSignal.refresh(true).value.ofUnit(rotations)

    override val angularVelocity: AngularVelocity
        get() = velocitySignal.refresh(true).value.ofUnit(rotations/seconds)
}




/**
 * Creates an instance of a [ChargerTalonFX] through a "[configure]" lambda function,
 * which has the context of a [ChargerTalonFXConfiguration].
 *
 * ```
 * // example
 * val motor = ChargerTalonFX(canId = 6){ feedbackRemoteSensorId = 5 }
 */
public inline fun ChargerTalonFX(
    deviceId: Int,
    canBus: String = "rio",
    factoryDefault: Boolean = true,
    configure: ChargerTalonFXConfiguration.() -> Unit
): ChargerTalonFX = ChargerTalonFX(
    deviceId, canBus, factoryDefault,
    ChargerTalonFXConfiguration().apply(configure)
)




/**
 * Represents a TalonFX motor controller.
 * Includes everything in the CTRE TalonFX class,
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * Creating an instance of this class factory will factory default the motor;
 * set factoryDefault = false to turn this off.
 *
 * @see com.ctre.phoenix6.hardware.TalonFX
 * @see ChargerTalonFXConfiguration
 */
public class ChargerTalonFX(
    deviceId: Int,
    canBus: String = "rio",
    factoryDefault: Boolean = true,
    configuration: ChargerTalonFXConfiguration? = null
): TalonFX(deviceId, canBus), SmartEncoderMotorController, HardwareConfigurable<ChargerTalonFXConfiguration> {

    private val forwardTalonFXFollowers: MutableSet<TalonFX> = mutableSetOf()
    private val invertedTalonFXFollowers: MutableSet<TalonFX> = mutableSetOf()
    private val otherFollowers: MutableSet<SmartEncoderMotorController> = mutableSetOf()

    private val allConfigErrors: LinkedHashSet<StatusCode> = linkedSetOf()
    private var configAppliedProperly = true
    private fun StatusCode.updateConfigStatus(): StatusCode {
        if (this != StatusCode.OK){
            if (RobotBase.isSimulation()){
                println("A Phoenix Device did not configure properly; however, this was ignored because the code is running in simulation.")
            }else{
                allConfigErrors.add(this)
                configAppliedProperly = false
            }
        }
        return this
    }

    init{
        val baseConfig = TalonFXConfiguration()

        if (!factoryDefault){
            configurator.refresh(baseConfig)
        }else{
            // an empty CTRE TalonFXConfiguration will factory default the motor if applied.
            println("TalonFX will factory default.")
        }

        if (configuration != null){
            configure(configuration, baseConfig)
        }else{
            configure(ChargerTalonFXConfiguration(), baseConfig)
        }
    }


    /**
     * The encoder of the TalonFX.
     */
    override val encoder: TalonFXEncoderAdapter =
        TalonFXEncoderAdapter(this)


    private val voltageSignal = supplyVoltage
    private val currentSignal = statorCurrent
    private val tempSignal = deviceTemp

    override val appliedVoltage: Voltage
        get() = voltageSignal.refresh(true).value.ofUnit(volts)

    override val appliedCurrent: Current
        get() = currentSignal.refresh(true).value.ofUnit(amps)

    override val tempCelsius: Double
        get() = tempSignal.refresh(true).value


    private val defaultFollowRequest = Follower(deviceID, false)
    private val invertFollowRequest = Follower(deviceID, true)

    /**
     * Adds follower motors to this TalonFX that mirror this motor's direction,
     * regardless of invert.
     *
     * See [withInvertedFollowers] if having followers
     * which run in the opposite direction of the master is desired.
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    public fun withFollowers(vararg followers: SmartEncoderMotorController){
        val thisInverted = this.inverted
        followers.forEach{
            if (it is TalonFX){
                forwardTalonFXFollowers.add(it)
            }else{
                it.inverted = thisInverted
                otherFollowers.add(it)
            }
        }
    }


    /**
     * Adds follower motors to this TalonFX that run in the opposite direction of this motor,
     * regardless of invert.
     *
     * See [withFollowers] if inverting direction is not desired.
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    public fun withInvertedFollowers(vararg followers: SmartEncoderMotorController){
        val thisInverted = this.inverted
        followers.forEach{
            if (it is TalonFX){
                invertedTalonFXFollowers.add(it)
            }else{
                it.inverted = !thisInverted
                otherFollowers.add(it)
            }
        }
    }


    private fun runFollowing(){
        forwardTalonFXFollowers.forEach{ it.setControl(defaultFollowRequest) }
        invertedTalonFXFollowers.forEach{ it.setControl(invertFollowRequest) }
    }

    override fun setControl(request: VoltageOut): StatusCode {
        runFollowing()
        otherFollowers.forEach{
            it.setVoltage(request.Output)
        }
        return super.setControl(request)
    }

    override fun setControl(request: DutyCycleOut): StatusCode {
        runFollowing()
        otherFollowers.forEach{
            it.set(request.Output)
        }
        return super.setControl(request) // returns a status code
    }

    override fun setControl(request: NeutralOut): StatusCode {
        runFollowing()
        otherFollowers.forEach {
            it.stopMotor()
        }
        return super.setControl(request) // returns a status code
    }





    private val currentSlotConfigs = Slot0Configs()
    private val velocityRequest = VelocityVoltage(0.0).also{ it.Slot = 0 }
    private val positionRequest = PositionVoltage(0.0).also{it.Slot = 0 }
    private var isWrapping = false

    private fun currentPIDConstants() = PIDConstants(currentSlotConfigs.kP, currentSlotConfigs.kI, currentSlotConfigs.kD)

    private fun setPIDConstants(newConstants: PIDConstants){
        currentSlotConfigs.kP = newConstants.kP
        currentSlotConfigs.kI = newConstants.kI
        currentSlotConfigs.kD = newConstants.kD
    }

    override fun setAngularVelocity(
        target: AngularVelocity,
        pidConstants: PIDConstants,
        feedforwardConstants: AngularMotorFFConstants
    ) {
        var configHasChanged = false
        if (currentPIDConstants() != pidConstants){
            setPIDConstants(pidConstants)
            configHasChanged = true
        }
        if (currentSlotConfigs.kS != feedforwardConstants.kS.inUnit(volts) || currentSlotConfigs.kV != feedforwardConstants.kV.inUnit(volts * seconds / rotations)){
            currentSlotConfigs.kS = feedforwardConstants.kS.inUnit(volts)
            currentSlotConfigs.kV = feedforwardConstants.kV.inUnit(volts * seconds / rotations)
            configHasChanged = true
        }
        if (configHasChanged){
            configurator.apply(currentSlotConfigs)
            println("PID status has been updated.")
        }
        velocityRequest.Velocity = target.inUnit(rotations/seconds)
        setControl(velocityRequest)
        runFollowing()
        otherFollowers.forEach{
            it.setAngularVelocity(target, pidConstants, feedforwardConstants)
        }
    }

    override fun setAngularPosition(
        target: Angle,
        pidConstants: PIDConstants,
        continuousWrap: Boolean,
        extraVoltage: Voltage
    ) {

        if (currentPIDConstants() != pidConstants){
            setPIDConstants(pidConstants)
            configurator.apply(currentSlotConfigs)
            println("PID status for Talon FX has been updated.")
        }

        if (isWrapping != continuousWrap){
            configurator.apply(
                ClosedLoopGeneralConfigs().apply{ContinuousWrap = continuousWrap}
            )
            isWrapping = continuousWrap
            println("Closed Loop status for TalonFX has been updated.")
        }

        if (isWrapping){
            positionRequest.Position = target
                .inputModulus((-0.5).rotations..0.5.rotations)
                .inUnit(rotations)
        }else{
            positionRequest.Position = target.inUnit(rotations)
        }
        positionRequest.FeedForward = extraVoltage.inUnit(volts)
        setControl(positionRequest)
        runFollowing()
        otherFollowers.forEach{
            it.setAngularPosition(target, pidConstants, continuousWrap, extraVoltage, this.encoder)
        }
    }


    override fun configure(configuration: ChargerTalonFXConfiguration) {
        // for a CTRE TalonFXConfiguration,
        // calling configurator.apply(configuration) will cause all configurations not explicitly specified
        // to revert to the factory default.
        // in contrast, ChargerTalonFXConfiguration has all values default to null,
        // where "null" is an untouched configuration(aka preserved from previous configurations).
        // thus, to acheive this functionality, a CTRE configuration must be refreshed FIRST
        // before changes are applied and the configuration is configured.
        val baseTalonFXConfig = TalonFXConfiguration()
        configurator.refresh(baseTalonFXConfig)
        configure(configuration, baseTalonFXConfig)
    }


    public fun configure(configuration: ChargerTalonFXConfiguration, baseTalonFXConfiguration: TalonFXConfiguration){
        configAppliedProperly = true
        safeConfigure(
            deviceName = "ChargerTalonFX(id = $deviceID)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ){
            allConfigErrors.clear()
            applyChanges(baseTalonFXConfiguration, configuration)
            configurator.apply(baseTalonFXConfiguration,0.02).updateConfigStatus()

            configuration.apply{
                positionUpdateFrequency?.let{
                    position.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }

                velocityUpdateFrequency?.let{
                    velocity.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }

                motorOutputUpdateFrequency?.let{
                    supplyVoltage.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                    dutyCycle.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }

                currentUpdateFrequency?.let{
                    torqueCurrent.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                    supplyCurrent.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                    statorCurrent.setUpdateFrequency(it.inUnit(hertz)).updateConfigStatus()
                }
            }
            return@safeConfigure configAppliedProperly
        }
    }

}






/**
 * A data class representing all possible configuration parameters
 * of a ChargerTalonFX.
 *
 * Identical to CTRE's TalonFXConfiguration for Phoenix v6, except for a couple of changes:
 *
 * 1. This configuration is designed to "apply" changes onto the existing configuration instead of overriding them.
 *    Thus, a configuration with the value null actually means a configuration that is not modified
 *    compared to the current one. Since motors are factory-defaulted upon initialization using convenience functions,
 *    it is not nessecary to have overriding configuration.
 *
 *
 * 2. PID / Motion magic configuration is removed, and replaced with FeedbackMotorController functionality.
 *
 * @see ChargerTalonFX
 */
public data class ChargerTalonFXConfiguration(
    // audio configs
    var beepOnBoot: Boolean? = null,

    // Closed Loop Ramps Configs
    var dutyCycleClosedLoopRampPeriod: Time? = null,
    var torqueClosedLoopRampPeriod: Time? = null,
    var voltageClosedLoopRampPeriod: Time? = null,

    // Open loop ramp configs
    var dutyCycleOpenLoopRampPeriod: Time? = null,
    var torqueOpenLoopRampPeriod: Time? = null,
    var voltageOpenLoopRampPeriod: Time? = null,

    // Current Limit Configs
    var statorCurrentLimitEnable: Boolean? = null,
    var supplyCurrentLimitEnable: Boolean? = null,
    var statorCurrentLimit: Current? = null,
    var supplyCurrentLimit: Current? = null,
    var supplyCurrentThreshold: Current? = null,
    var supplyTimeThreshold: Time? = null,

    // feedback configs
    var feedbackRemoteSensorID: Int? = null,
    var feedbackRotorOffset: Angle? = null,
    var feedbackSensorSource: FeedbackSensorSourceValue? = null,
    var rotorToSensorRatio: Double? = null,
    var sensorToMechanismRatio: Double? = null,

    // Hardware Limit Switch Configs
    var forwardLimitEnable: Boolean? = null,
    var forwardLimitAutosetPositionEnable: Boolean? = null,
    var forwardLimitAutosetPositionValue: Angle? = null,
    var forwardLimitRemoteSensorID: Int? = null,
    var forwardLimitSource: ForwardLimitSourceValue? = null,
    var forwardLimitType: ForwardLimitTypeValue? = null,

    var reverseLimitEnable: Boolean? = null,
    var reverseLimitAutosetPositionEnable: Boolean? = null,
    var reverseLimitAutosetPositionValue: Angle? = null,
    var reverseLimitRemoteSensorID: Int? = null,
    var reverseLimitSource: ReverseLimitSourceValue? = null,
    var reverseLimitType: ReverseLimitTypeValue? = null,

    // Motor Output Configs
    var neutralMode: NeutralModeValue? = null,
    var inverted: Boolean? = null,
    var dutyCycleNeutralDeadband: Double? = null,
    var peakForwardDutyCycle: Double? = null,
    var peakReverseDutyCycle: Double? = null,

    // Software Limit Switch Configs
    var forwardSoftLimitEnable: Boolean? = null,
    var reverseSoftLimitEnable: Boolean? = null,
    var forwardSoftLimitThreshold: Angle? = null,
    var reverseSoftLimitThreshold: Angle? = null,

    // Torque Current Configs
    var peakForwardTorqueCurrent: Current? = null,
    var peakReverseTorqueCurrent: Current? = null,
    var torqueNeutralDeadband: Current? = null,

    // Voltage Configs
    var peakForwardVoltage: Voltage? = null,
    var peakReverseVoltage: Voltage? = null,
    var supplyVoltageTimeConstant: Time? = null,

    // update frequency configs(not included in TalonFXConfiguration)
    var positionUpdateFrequency: Frequency? = null,
    var velocityUpdateFrequency: Frequency? = null,
    var motorOutputUpdateFrequency: Frequency? = null,
    var currentUpdateFrequency: Frequency? = null,

    // Differential Constants configs
    var peakDifferentialTorqueCurrent: Current? = null,
    var peakDifferentialVoltage: Voltage? = null,
    var peakDifferentialDutyCycle: Double? = null,

    // Differential Sensors configs
    var differentialTalonFXSensorID: Int? = null,
    var differentialRemoteSensorID: Int? = null,
    var differentialSensorSource: DifferentialSensorSourceValue? = null
): HardwareConfiguration

internal fun applyChanges(ctreConfig: TalonFXConfiguration, chargerConfig: ChargerTalonFXConfiguration): TalonFXConfiguration{
    ctreConfig.apply{
        chargerConfig.beepOnBoot?.let{
            Audio.BeepOnBoot = it
        }

        CurrentLimits.apply{
            chargerConfig.statorCurrentLimitEnable?.let{ StatorCurrentLimitEnable = it }
            chargerConfig.statorCurrentLimit?.let{ StatorCurrentLimit = it.inUnit(amps) }
            chargerConfig.supplyCurrentLimit?.let{ SupplyCurrentLimit = it.inUnit(amps) }
            chargerConfig.supplyCurrentLimitEnable?.let{ SupplyCurrentLimitEnable = it }
            chargerConfig.supplyCurrentThreshold?.let{ SupplyCurrentThreshold = it.inUnit(amps) }
            chargerConfig.supplyTimeThreshold?.let{ SupplyTimeThreshold = it.inUnit(seconds) }
        }

        ClosedLoopRamps.apply{
            chargerConfig.dutyCycleClosedLoopRampPeriod?.let{ DutyCycleClosedLoopRampPeriod = it.inUnit(seconds) }
            chargerConfig.torqueClosedLoopRampPeriod?.let{ TorqueClosedLoopRampPeriod = it.inUnit(seconds) }
            chargerConfig.voltageClosedLoopRampPeriod?.let{ VoltageClosedLoopRampPeriod = it.inUnit(seconds) }
        }
        OpenLoopRamps.apply{
            chargerConfig.dutyCycleOpenLoopRampPeriod?.let{ DutyCycleOpenLoopRampPeriod = it.inUnit(seconds) }
            chargerConfig.torqueOpenLoopRampPeriod?.let{ TorqueOpenLoopRampPeriod = it.inUnit(seconds) }
            chargerConfig.voltageOpenLoopRampPeriod?.let{ VoltageOpenLoopRampPeriod = it.inUnit(seconds) }
        }

        Feedback.apply{
            chargerConfig.feedbackRemoteSensorID?.let{ FeedbackRemoteSensorID = it }
            chargerConfig.feedbackRotorOffset?.let{ FeedbackRotorOffset = it.inUnit(rotations) }
            chargerConfig.feedbackSensorSource?.let{ FeedbackSensorSource = it }
            chargerConfig.rotorToSensorRatio?.let{ RotorToSensorRatio = it }
            chargerConfig.sensorToMechanismRatio?.let{ SensorToMechanismRatio = it }
        }

        HardwareLimitSwitch.apply{
            chargerConfig.forwardLimitEnable?.let{ ForwardLimitEnable = it }
            chargerConfig.forwardLimitAutosetPositionEnable?.let{ ForwardLimitAutosetPositionEnable = it }
            chargerConfig.forwardLimitAutosetPositionValue?.let{ ForwardLimitAutosetPositionValue = it.inUnit(rotations) }
            chargerConfig.forwardLimitRemoteSensorID?.let{ForwardLimitRemoteSensorID = it}
            chargerConfig.forwardLimitSource?.let{ ForwardLimitSource = it }
            chargerConfig.forwardLimitType?.let{ ForwardLimitType = it }


            chargerConfig.reverseLimitEnable?.let{ ReverseLimitEnable = it }
            chargerConfig.reverseLimitAutosetPositionEnable?.let{ ReverseLimitAutosetPositionEnable = it }
            chargerConfig.reverseLimitAutosetPositionValue?.let{ ReverseLimitAutosetPositionValue = it.inUnit(rotations) }
            chargerConfig.reverseLimitRemoteSensorID?.let{ReverseLimitRemoteSensorID = it}
            chargerConfig.reverseLimitSource?.let{ ReverseLimitSource = it }
            chargerConfig.reverseLimitType?.let{ ReverseLimitType = it }
        }

        MotorOutput.apply{
            chargerConfig.inverted?.let{ Inverted = if (it) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive }
            chargerConfig.neutralMode?.let{ NeutralMode = it }
            chargerConfig.dutyCycleNeutralDeadband?.let{ DutyCycleNeutralDeadband = it}
            chargerConfig.peakForwardDutyCycle?.let{PeakForwardDutyCycle = it}
            chargerConfig.peakReverseDutyCycle?.let{ PeakReverseDutyCycle = it }
        }

        SoftwareLimitSwitch.apply{
            chargerConfig.forwardSoftLimitEnable?.let{ ForwardSoftLimitEnable = it }
            chargerConfig.forwardSoftLimitThreshold?.let{ ForwardSoftLimitThreshold = it.inUnit(rotations) }
            chargerConfig.reverseSoftLimitEnable?.let{ ReverseSoftLimitEnable = it }
            chargerConfig.reverseSoftLimitThreshold?.let{ ReverseSoftLimitThreshold = it.inUnit(rotations) }
        }

        TorqueCurrent.apply{
            chargerConfig.peakForwardTorqueCurrent?.let{ PeakForwardTorqueCurrent = it.inUnit(amps) }
            chargerConfig.peakReverseTorqueCurrent?.let{ PeakReverseTorqueCurrent = it.inUnit(amps) }
            chargerConfig.torqueNeutralDeadband?.let{ TorqueNeutralDeadband = it.inUnit(amps) }
        }

        Voltage.apply{
            chargerConfig.peakForwardVoltage?.let{ PeakForwardVoltage = it.inUnit(volts) }
            chargerConfig.peakReverseVoltage?.let{ PeakReverseVoltage = it.inUnit(volts) }
            chargerConfig.supplyVoltageTimeConstant?.let{ SupplyVoltageTimeConstant = it.inUnit(seconds) }
        }

        DifferentialConstants.apply{
            chargerConfig.peakDifferentialTorqueCurrent?.let{
                PeakDifferentialTorqueCurrent = it.inUnit(amps)
            }
            chargerConfig.peakDifferentialVoltage?.let{
                PeakDifferentialVoltage = it.inUnit(volts)
            }
            chargerConfig.peakDifferentialDutyCycle?.let{
                PeakDifferentialDutyCycle = it
            }
        }

        DifferentialSensors.apply{
            chargerConfig.differentialTalonFXSensorID?.let{
                DifferentialTalonFXSensorID = it
            }
            chargerConfig.differentialRemoteSensorID?.let{
                DifferentialRemoteSensorID = it
            }
            chargerConfig.differentialSensorSource?.let{
                DifferentialSensorSource = it
            }
        }
    }
    return ctreConfig
}






