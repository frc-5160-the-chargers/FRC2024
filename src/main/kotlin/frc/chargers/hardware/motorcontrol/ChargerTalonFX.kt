@file:Suppress("unused")
package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.Encoder
import kotlin.math.PI


/**
 * A [TalonFX] motor controller that implements the [Motor] interface,
 * and performs periodic self-checking.
 *
 * Includes everything in the CTRE TalonFX class(accessed via the [base] property),
 * but has additional features to mesh better with the rest
 * of this library.
 *
 * Creating an instance of this class factory will factory default the motor;
 * set factoryDefault = false to turn this off.
 *
 * @see com.ctre.phoenix6.hardware.TalonFX
 */
class ChargerTalonFX(
    val deviceID: Int,
    canBus: String? = null,
    factoryDefault: Boolean = true,
    private val faultLogName: String? = null,
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

    private val setPosRequest = PositionVoltage(0.0).withSlot(0)
    private val setVelRequest = VelocityVoltage(0.0).withSlot(1)

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

    override var appliedVoltage: Voltage
        get() = voltageSignal.refresh(true).value.ofUnit(volts)
        set(value){
            base.setVoltage(value.siValue)
            nonTalonFXFollowers.forEach{ it.appliedVoltage = value }
        }

    override val statorCurrent: Current
        get() = currentSignal.refresh(true).value.ofUnit(amps)

    override val inverted: Boolean get() = base.inverted

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        require(positionPIDConfigured){" You must specify a positionPID value using the configure(positionPID = PIDConstants(p,i,d)) method. "}
        setPosRequest.Position = position.inUnit(rotations)
        setPosRequest.FeedForward = feedforward.inUnit(volts)
        base.setControl(setPosRequest)
        nonTalonFXFollowers.forEach{ it.setPositionSetpoint(position, feedforward) }
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        require(velocityPIDConfigured){" You must specify a velocityPID value using the configure(velocityPID = PIDConstants(p,i,d)) method. "}
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
        startingPosition: Angle?,
        positionPID: PIDConstants?,
        velocityPID: PIDConstants?,
        continuousInput: Boolean?
    ): ChargerTalonFX {
        val errors = mutableListOf<StatusCode>()
        fun StatusCode.bind(){ if (this != StatusCode.OK) errors.add(this) }
        for (i in 1..4) {
            if (inverted != null) {
                config.MotorOutput.Inverted = if (inverted) {
                    InvertedValue.Clockwise_Positive
                } else {
                    InvertedValue.CounterClockwise_Positive
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
            if (startingPosition != null) base.setPosition(startingPosition.inUnit(rotations)).bind()
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
                    startingPosition = startingPosition
                )
                when (follower) {
                    is ChargerTalonFX -> follower.base.setControl(Follower(this.deviceID, follower.inverted)).bind()
                    else -> nonTalonFXFollowers.add(follower)
                }
            }
            base.configurator.apply(config, 0.050).bind()

            val optimizeBusUtilization = optimizeUpdateRate == true
            if (optimizeBusUtilization) {
                base.optimizeBusUtilization()
                base.statorCurrent.setUpdateFrequency(50.0).bind()
                base.motorVoltage.setUpdateFrequency(50.0).bind()
            }
            for ((statusSignal, rate) in mapOf(base.position to positionUpdateRate, base.velocity to velocityUpdateRate)){
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
        DriverStation.reportError("ERROR: ${faultLogName ?: "ChargerTalonFX($deviceID)"} failed to configure. Errors: $errors", false)
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
                    if (faultSignal.refresh().value != false) HorseLog.logFault("$faultLogName: $faultMsg")
                }
                if (voltageSignal.refresh().status != StatusCode.OK) {
                    HorseLog.logFault("$faultLogName: Device is unreachable")
                }
            }
        }
    }
}
