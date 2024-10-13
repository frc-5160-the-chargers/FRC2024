@file:Suppress("unused")
package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.Encoder
import kotlin.math.PI


/**
 * A [TalonFX] motor controller that implements the [Motor] interface
 * and performs periodic self-checking.
 *
 * The [base] property allows for the access of the base [TalonFX] instance.
 */
class ChargerTalonFX(
    val deviceID: Int,
    canBus: String? = null,
    factoryDefault: Boolean = true,
    private val faultLogName: String? = null,
    /**
     * Note: this requires the phoenix pro subscription; ask daniel for more details
     */
    private val fusedCANCoder: ChargerCANcoder? = null
): Motor {
    /**
     * The base [TalonFX] instance.
     */
    val base: TalonFX = if (canBus == null) TalonFX(deviceID) else TalonFX(deviceID, canBus)

    private val config = TalonFXConfiguration()
    private val nonTalonFXFollowers = mutableListOf<Motor>()

    private val positionSignal = base.position
    private val velocitySignal = base.velocity
    private val voltageSignal = base.motorVoltage
    private val currentSignal = base.statorCurrent

    private val voltOutRequest = VoltageOut(0.0).withEnableFOC(true)
    private val setPosRequest = PositionVoltage(0.0).apply {
        Slot = 0
        EnableFOC = true
    }
    private val setVelRequest = VelocityVoltage(0.0).apply {
        Slot = 1
        EnableFOC = true
    }

    private var positionPIDConfigured = false
    private var velocityPIDConfigured = false

    init {
        if (factoryDefault || fusedCANCoder != null) {
            if (!factoryDefault) base.configurator.refresh(config)
            if (fusedCANCoder != null) {
                config.Feedback.FeedbackRemoteSensorID = fusedCANCoder.deviceID
                config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }
            base.configurator.apply(config)
        }
    }

    /**
     * The encoder of the TalonFX.
     */
    override val encoder: Encoder = TalonFXEncoderAdapter()
    private inner class TalonFXEncoderAdapter: Encoder {
        override val angularPosition: Angle
            get() = positionSignal.refresh(true).value.ofUnit(rotations)

        override val angularVelocity: AngularVelocity
            get() = velocitySignal.refresh(true).value.ofUnit(rotations/seconds)
    }

    override var voltageOut: Voltage
        get() = voltageSignal.refresh(true).value.ofUnit(volts)
        set(value){
            voltOutRequest.Output = value.siValue
            base.setControl(voltOutRequest)
            nonTalonFXFollowers.forEach{ it.voltageOut = value }
        }

    override val statorCurrent get() = currentSignal.refresh(true).value.ofUnit(amps)

    override val inverted get() = base.inverted

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        if (!positionPIDConfigured) {
            alertPositionPIDErr()
            return
        } else if (abs(position - this.encoder.angularPosition) < 0.5.degrees) {
            base.setVoltage(0.0)
            return
        }
        setPosRequest.Position = position.inUnit(rotations)
        setPosRequest.FeedForward = feedforward.inUnit(volts)
        base.setControl(setPosRequest)
        nonTalonFXFollowers.forEach{ it.setPositionSetpoint(position, feedforward) }
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        if (!velocityPIDConfigured) {
            alertVelocityPIDErr()
            return
        }
        setVelRequest.Velocity = velocity.inUnit(rotations / seconds)
        setVelRequest.FeedForward = feedforward.inUnit(volts)
        base.setControl(setVelRequest)
        nonTalonFXFollowers.forEach{ it.setVelocitySetpoint(velocity, feedforward) }
    }

    override fun configure(
        inverted: Boolean?,
        brakeWhenIdle: Boolean?,
        rampRate: Time?,
        statorCurrentLimit: Current?,
        followerMotors: List<Motor>,
        positionUpdateRate: Frequency?,
        velocityUpdateRate: Frequency?,
        optimizeUpdateRate: Boolean?,
        gearRatio: Double?,
        currentPosition: Angle?,
        positionPID: PIDConstants?,
        velocityPID: PIDConstants?,
        continuousInput: Boolean?
    ): ChargerTalonFX {
        val errors = mutableListOf<StatusCode>()
        fun StatusCode.bind(){ if (this != StatusCode.OK) errors.add(this) }
        for (i in 1..4) {
            config.MotorOutput.apply {
                when (brakeWhenIdle) {
                    true -> NeutralMode = NeutralModeValue.Brake
                    false -> NeutralMode = NeutralModeValue.Coast
                    null -> {}
                }
                when (inverted) {
                    true -> Inverted = InvertedValue.Clockwise_Positive
                    false -> Inverted = InvertedValue.CounterClockwise_Positive
                    null -> {}
                }
            }
            if (rampRate != null){
                config.OpenLoopRamps.apply {
                    TorqueOpenLoopRampPeriod = rampRate.inUnit(seconds)
                    VoltageOpenLoopRampPeriod = rampRate.inUnit(seconds)
                    DutyCycleOpenLoopRampPeriod = rampRate.inUnit(seconds)
                }
                config.ClosedLoopRamps.apply {
                    TorqueClosedLoopRampPeriod = rampRate.inUnit(seconds)
                    VoltageClosedLoopRampPeriod = rampRate.inUnit(seconds)
                    DutyCycleClosedLoopRampPeriod = rampRate.inUnit(seconds)
                }
            }
            if (statorCurrentLimit != null) {
                config.CurrentLimits.StatorCurrentLimitEnable = true
                config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.inUnit(amps)
            }
            if (currentPosition != null) base.setPosition(currentPosition.inUnit(rotations)).bind()
            if (gearRatio != null) {
                if (fusedCANCoder != null) {
                    config.Feedback.RotorToSensorRatio = gearRatio
                } else {
                    config.Feedback.SensorToMechanismRatio = gearRatio
                }
            }
            if (continuousInput != null) config.ClosedLoopGeneral.ContinuousWrap = continuousInput
            // 2 * PI makes it so that the PID gains are optimized off of radians and not rotations
            if (positionPID != null) {
                positionPIDConfigured = true
                config.Slot0.apply {
                    kP = positionPID.kP * (2 * PI)
                    kI = positionPID.kI * (2 * PI)
                    kD = positionPID.kD * (2 * PI)
                }
            }
            if (velocityPID != null) {
                velocityPIDConfigured = true
                config.Slot1.apply {
                    kP = velocityPID.kP * (2 * PI)
                    kI = velocityPID.kI * (2 * PI)
                    kD = velocityPID.kD * (2 * PI)
                }
            }
            for (follower in followerMotors){
                follower.configure(
                    positionPID = positionPID,
                    velocityPID = velocityPID,
                    gearRatio = gearRatio,
                    currentPosition = currentPosition
                )
                when (follower) {
                    is ChargerTalonFX -> follower.base.setControl(Follower(this.deviceID, follower.inverted)).bind()
                    else -> nonTalonFXFollowers.add(follower)
                }
            }
            base.configurator.apply(config, 0.1).bind()

            val optimizeBusUtilization = optimizeUpdateRate == true
            if (optimizeBusUtilization) {
                base.optimizeBusUtilization()
                currentSignal.setUpdateFrequency(50.0).bind()
                voltageSignal.setUpdateFrequency(50.0).bind()
            }
            for ((statusSignal, rate) in mapOf(positionSignal to positionUpdateRate, velocitySignal to velocityUpdateRate)){
                if (rate != null) {
                    statusSignal.setUpdateFrequency(rate.inUnit(hertz)).bind()
                } else if (optimizeBusUtilization) {
                    // optimizeBusUtilization will turn the position and velocity signals off by default, so we have to re-enable them
                    statusSignal.setUpdateFrequency(50.0).bind()
                }
            }
            if (errors.isEmpty()) return this
            errors.clear()
        }
        HorseLog.logError(
            "${faultLogName ?: "ChargerTalonFX($deviceID)"} failed to configure.",
            "Errors: $errors"
        )
        return this
    }

    fun limitSupplyCurrent(
        limit: Current,
        highLimit: Current? = null,
        highLimitAllowedFor: Time? = null
    ): ChargerTalonFX {
        val configs = CurrentLimitsConfigs()
        base.configurator.refresh(configs, 0.05)
        configs.apply {
            SupplyCurrentLimitEnable = true
            SupplyCurrentLimit = limit.inUnit(amps)
            if (highLimit != null) SupplyCurrentThreshold = highLimit.inUnit(amps)
            if (highLimitAllowedFor != null) SupplyTimeThreshold = highLimitAllowedFor.inUnit(seconds)
        }
        base.configurator.apply(configs, 0.1)
        return this
    }

    private val faultSignalToMsg = mapOf(
        base.fault_Hardware to "Hardware failure detected",
        base.fault_DeviceTemp to "Device temp exceeded limit",
        base.fault_BootDuringEnable to "Device booted when enabled",
        base.fault_FusedSensorOutOfSync to "Fused CANcoder out of sync",
        base.fault_OverSupplyV to "Voltage exceeded limit",
        base.fault_ProcTemp to "Processor is overheating",
        base.fault_UnlicensedFeatureInUse to "Unlicensed feature in use(git Phoenix Pro Pls)",
        base.fault_UnstableSupplyV to "Supply voltage unstable",
    )

    init {
        if (faultLogName != null) {
            ChargerRobot.runPeriodicAtPeriod(1.seconds){
                for ((faultSignal, faultMsg) in faultSignalToMsg){
                    // != false prevents null
                    if (faultSignal.refresh().value != false) HorseLog.logError("(Motor Fault) $faultLogName", faultMsg)
                }
                if (voltageSignal.refresh().status != StatusCode.OK) {
                    HorseLog.logError("(Motor Fault) $faultLogName", "Device is Unreachable")
                }
            }
        }
    }
}
