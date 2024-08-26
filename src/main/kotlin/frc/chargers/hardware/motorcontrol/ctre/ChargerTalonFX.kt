@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.ctre

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.hardware.configuration.ConfigurableHardware
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.math.inputModulus


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
): TalonFX(deviceId, canBus), Motor, ConfigurableHardware<ChargerTalonFXConfiguration> {
    private val talonFXFollowers: MutableSet<TalonFX> = mutableSetOf()
    private val otherFollowers: MutableSet<Motor> = mutableSetOf()

    private val voltageSignal = supplyVoltage
    private val currentSignal = getStatorCurrent()
    private val tempSignal = deviceTemp
    private val followRequest = Follower(deviceID, false)

    init{
        if (factoryDefault){
            applyConfiguration(configuration ?: ChargerTalonFXConfiguration(), factoryDefault = true)
        }else{
            if (configuration != null){
                applyConfiguration(configuration, factoryDefault = false)
            }
        }
    }

    /**
     * The encoder of the TalonFX.
     */
    override val encoder: ResettableEncoder = TalonFXEncoderAdapter()
    private inner class TalonFXEncoderAdapter: ResettableEncoder {
        private val positionSignal = this@ChargerTalonFX.position
        private val velocitySignal = this@ChargerTalonFX.velocity

        override fun setZero(newZero: Angle) {
            val errorCode = this@ChargerTalonFX.setPosition(newZero.inUnit(rotations))
            if (errorCode != StatusCode.OK){
                error("When attempting to zero a talon fx, an error occurred: $errorCode")
            }
        }

        override val angularPosition: Angle
            get() = positionSignal.refresh(true).value.ofUnit(rotations)

        override val angularVelocity: AngularVelocity
            get() = velocitySignal.refresh(true).value.ofUnit(rotations/seconds)
    }

    override var hasInvert: Boolean
        get() = getInverted()
        set(value) = setInverted(value)

    override var appliedVoltage: Voltage
        get() = voltageSignal.refresh(true).value.ofUnit(volts) * get()
        set(value){ setVoltage(value.siValue) }

    override val statorCurrent: Current
        get() = currentSignal.refresh(true).value.ofUnit(amps)

    /**
     * Adds follower motors to this TalonFX that mirror this motor's direction,
     * regardless of invert.
     *
     * Do not try and access the individual motors passed into this function,
     * as this can lead to unexpected results. To configure followers,
     * it is recommended to use an [apply] or [also] block, or use ChargerLib's inline configuration to do so.
     */
    override fun withFollowers(vararg followers: Motor): Motor {
        val nonTalonFXFollowers = mutableListOf<Motor>()
        for (follower in followers){
            if (follower is TalonFX){
                talonFXFollowers.add(follower)
            }else{
                nonTalonFXFollowers.add(follower)
            }
        }
        return super.withFollowers(*nonTalonFXFollowers.toTypedArray())
    }

    private fun runFollowing(){
        talonFXFollowers.forEach{ it.setControl(followRequest) }
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

    override fun setBrakeMode(shouldBrake: Boolean){
        setNeutralMode(if (shouldBrake) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun setVelocitySetpoint(
        rawVelocity: AngularVelocity,
        pidConstants: PIDConstants,
        feedforward: Voltage
    ) {
        if (currentPIDConstants() != pidConstants){
            setPIDConstants(pidConstants)
            configurator.apply(currentSlotConfigs)
            println("PID status has been updated.")
        }
        velocityRequest.Velocity = rawVelocity.inUnit(rotations/seconds)
        velocityRequest.FeedForward = feedforward.inUnit(volts)
        setControl(velocityRequest)
        runFollowing()
        otherFollowers.forEach{
            it.setVelocitySetpoint(rawVelocity, pidConstants, feedforward)
        }
    }

    override fun setPositionSetpoint(
        rawPosition: Angle,
        pidConstants: PIDConstants,
        continuousInput: Boolean,
        feedforward: Voltage
    ) {
        if (currentPIDConstants() != pidConstants){
            setPIDConstants(pidConstants)
            configurator.apply(currentSlotConfigs)
            println("PID status for Talon FX has been updated.")
        }

        if (isWrapping != continuousInput){
            configurator.apply(
                ClosedLoopGeneralConfigs().apply{ContinuousWrap = continuousInput}
            )
            isWrapping = continuousInput
            println("Closed Loop status for TalonFX has been updated.")
        }

        if (isWrapping){
            positionRequest.Position = rawPosition
                .inputModulus((-0.5).rotations..0.5.rotations)
                .inUnit(rotations)
        }else{
            positionRequest.Position = rawPosition.inUnit(rotations)
        }
        positionRequest.FeedForward = feedforward.inUnit(volts)
        setControl(positionRequest)
        runFollowing()
        otherFollowers.forEach{
            it.setPositionSetpoint(rawPosition, pidConstants, continuousInput, feedforward)
        }
    }

    override fun configure(configuration: ChargerTalonFXConfiguration) =
        applyConfiguration(configuration, factoryDefault = false)

    private fun applyConfiguration(configuration: ChargerTalonFXConfiguration, factoryDefault: Boolean) {
        val ctreConfig = TalonFXConfiguration()
        if (!factoryDefault){
            configurator.refresh(ctreConfig)
        }
        // I know, this looks like a stupidly large amount of code.
        // As a gist, this code basically takes all the configs from the ChargerTalonFXConfiguration
        // and applies them onto a regular TalonFXConfiguration.
        ctreConfig.apply{
            configuration.beepOnBoot?.let{
                Audio.BeepOnBoot = it
            }

            CurrentLimits.apply{
                configuration.statorCurrentLimitEnable?.let{ StatorCurrentLimitEnable = it }
                configuration.statorCurrentLimit?.let{ StatorCurrentLimit = it.inUnit(amps) }
                configuration.supplyCurrentLimit?.let{ SupplyCurrentLimit = it.inUnit(amps) }
                configuration.supplyCurrentLimitEnable?.let{ SupplyCurrentLimitEnable = it }
                configuration.supplyCurrentThreshold?.let{ SupplyCurrentThreshold = it.inUnit(amps) }
                configuration.supplyTimeThreshold?.let{ SupplyTimeThreshold = it.inUnit(seconds) }
            }

            ClosedLoopRamps.apply{
                configuration.dutyCycleClosedLoopRampPeriod?.let{ DutyCycleClosedLoopRampPeriod = it.inUnit(seconds) }
                configuration.torqueClosedLoopRampPeriod?.let{ TorqueClosedLoopRampPeriod = it.inUnit(seconds) }
                configuration.voltageClosedLoopRampPeriod?.let{ VoltageClosedLoopRampPeriod = it.inUnit(seconds) }
            }
            OpenLoopRamps.apply{
                configuration.dutyCycleOpenLoopRampPeriod?.let{ DutyCycleOpenLoopRampPeriod = it.inUnit(seconds) }
                configuration.torqueOpenLoopRampPeriod?.let{ TorqueOpenLoopRampPeriod = it.inUnit(seconds) }
                configuration.voltageOpenLoopRampPeriod?.let{ VoltageOpenLoopRampPeriod = it.inUnit(seconds) }
            }

            Feedback.apply{
                configuration.feedbackRemoteSensorID?.let{ FeedbackRemoteSensorID = it }
                configuration.feedbackRotorOffset?.let{ FeedbackRotorOffset = it.inUnit(rotations) }
                configuration.feedbackSensorSource?.let{ FeedbackSensorSource = it }
                configuration.rotorToSensorRatio?.let{ RotorToSensorRatio = it }
                configuration.sensorToMechanismRatio?.let{ SensorToMechanismRatio = it }
            }

            HardwareLimitSwitch.apply{
                configuration.forwardLimitEnable?.let{ ForwardLimitEnable = it }
                configuration.forwardLimitAutosetPositionEnable?.let{ ForwardLimitAutosetPositionEnable = it }
                configuration.forwardLimitAutosetPositionValue?.let{ ForwardLimitAutosetPositionValue = it.inUnit(rotations) }
                configuration.forwardLimitRemoteSensorID?.let{ForwardLimitRemoteSensorID = it}
                configuration.forwardLimitSource?.let{ ForwardLimitSource = it }
                configuration.forwardLimitType?.let{ ForwardLimitType = it }


                configuration.reverseLimitEnable?.let{ ReverseLimitEnable = it }
                configuration.reverseLimitAutosetPositionEnable?.let{ ReverseLimitAutosetPositionEnable = it }
                configuration.reverseLimitAutosetPositionValue?.let{ ReverseLimitAutosetPositionValue = it.inUnit(rotations) }
                configuration.reverseLimitRemoteSensorID?.let{ReverseLimitRemoteSensorID = it}
                configuration.reverseLimitSource?.let{ ReverseLimitSource = it }
                configuration.reverseLimitType?.let{ ReverseLimitType = it }
            }

            MotorOutput.apply{
                configuration.inverted?.let{ Inverted = if (it) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive }
                configuration.neutralMode?.let{ NeutralMode = it }
                configuration.dutyCycleNeutralDeadband?.let{ DutyCycleNeutralDeadband = it}
                configuration.peakForwardDutyCycle?.let{PeakForwardDutyCycle = it}
                configuration.peakReverseDutyCycle?.let{ PeakReverseDutyCycle = it }
            }

            SoftwareLimitSwitch.apply{
                configuration.forwardSoftLimitEnable?.let{ ForwardSoftLimitEnable = it }
                configuration.forwardSoftLimitThreshold?.let{ ForwardSoftLimitThreshold = it.inUnit(rotations) }
                configuration.reverseSoftLimitEnable?.let{ ReverseSoftLimitEnable = it }
                configuration.reverseSoftLimitThreshold?.let{ ReverseSoftLimitThreshold = it.inUnit(rotations) }
            }

            TorqueCurrent.apply{
                configuration.peakForwardTorqueCurrent?.let{ PeakForwardTorqueCurrent = it.inUnit(amps) }
                configuration.peakReverseTorqueCurrent?.let{ PeakReverseTorqueCurrent = it.inUnit(amps) }
                configuration.torqueNeutralDeadband?.let{ TorqueNeutralDeadband = it.inUnit(amps) }
            }

            Voltage.apply{
                configuration.peakForwardVoltage?.let{ PeakForwardVoltage = it.inUnit(volts) }
                configuration.peakReverseVoltage?.let{ PeakReverseVoltage = it.inUnit(volts) }
                configuration.supplyVoltageTimeConstant?.let{ SupplyVoltageTimeConstant = it.inUnit(seconds) }
            }

            DifferentialConstants.apply{
                configuration.peakDifferentialTorqueCurrent?.let{
                    PeakDifferentialTorqueCurrent = it.inUnit(amps)
                }
                configuration.peakDifferentialVoltage?.let{
                    PeakDifferentialVoltage = it.inUnit(volts)
                }
                configuration.peakDifferentialDutyCycle?.let{
                    PeakDifferentialDutyCycle = it
                }
            }

            DifferentialSensors.apply{
                configuration.differentialTalonFXSensorID?.let{
                    DifferentialTalonFXSensorID = it
                }
                configuration.differentialRemoteSensorID?.let{
                    DifferentialRemoteSensorID = it
                }
                configuration.differentialSensorSource?.let{
                    DifferentialSensorSource = it
                }
            }
        }

        configurator.apply(ctreConfig, 0.02)

        configuration.apply{
            positionUpdateFrequency?.let{
                position.setUpdateFrequency(it.inUnit(hertz))
            }

            velocityUpdateFrequency?.let{
                velocity.setUpdateFrequency(it.inUnit(hertz))
            }

            motorOutputUpdateFrequency?.let{
                voltageSignal.setUpdateFrequency(it.inUnit(hertz))
            }

            currentUpdateFrequency?.let{
                currentSignal.setUpdateFrequency(it.inUnit(hertz))
            }
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





