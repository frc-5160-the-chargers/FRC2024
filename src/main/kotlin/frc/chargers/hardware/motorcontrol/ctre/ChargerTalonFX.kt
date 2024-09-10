@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.ctre

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
import frc.chargers.framework.faultchecking.FaultChecking
import frc.chargers.framework.faultchecking.SubsystemFault
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.Encoder


/**
 * A [TalonFX] motor controller that implements the [Motor] and [FaultChecking] interfaces.
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
    private val fusedCANCoder: ChargerCANcoder? = null
): Motor, FaultChecking {
    /**
     * The base [TalonFX] instance.
     */
    val base: TalonFX = if (canBus == null) TalonFX(deviceID) else TalonFX(deviceID, canBus)

    private val config = TalonFXConfiguration()

    private val talonFXFollowers = mutableListOf<TalonFX>()
    private val invertedTalonFXFollowers = mutableListOf<TalonFX>()
    private val nonTalonFXFollowers = mutableListOf<Motor>()

    private val positionSignal = base.position
    private val velocitySignal = base.velocity
    private val voltageSignal = base.motorVoltage
    private val currentSignal = base.statorCurrent

    private val followRequest = Follower(deviceID, false)
    private val invertFollowRequest = Follower(deviceID, true)
    private val setPosRequest = PositionVoltage(0.0).withSlot(0)
    private val setVelRequest = VelocityVoltage(0.0).withSlot(1)

    private inline fun runFollowing(run: (Motor) -> Unit){
        talonFXFollowers.forEach{ it.setControl(followRequest) }
        invertedTalonFXFollowers.forEach { it.setControl(invertFollowRequest) }
        nonTalonFXFollowers.forEach(run)
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

    override var appliedVoltage: Voltage
        get() = voltageSignal.refresh(true).value.ofUnit(volts)
        set(value){
            base.setVoltage(value.siValue)
            runFollowing{ it.appliedVoltage = value }
        }

    override val statorCurrent: Current
        get() = currentSignal.refresh(true).value.ofUnit(amps)

    override val inverted: Boolean get() = base.inverted

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
        if (startingPosition != null) base.setPosition(startingPosition.inUnit(rotations))
        if (gearRatio != null) {
            if (fusedCANCoder != null) {
                config.Feedback.RotorToSensorRatio = gearRatio
            } else {
                config.Feedback.SensorToMechanismRatio = gearRatio
            }
        }
        if (continuousInput != null) config.ClosedLoopGeneral.ContinuousWrap = continuousInput
        if (positionPID != null) {
            positionPIDConfigured = true
            config.Slot0.apply {
                kP = positionPID.kP
                kI = positionPID.kI
                kD = positionPID.kD
            }
        }
        if (velocityPID != null) {
            velocityPIDConfigured = true
            config.Slot1.apply {
                kP = velocityPID.kP
                kI = velocityPID.kI
                kD = velocityPID.kD
            }
        }
        for (follower in followerMotors){
            if (follower is TalonFX){
                if (follower.inverted){
                    invertedTalonFXFollowers.add(follower)
                } else {
                    talonFXFollowers.add(follower)
                }
            }else{
                nonTalonFXFollowers.add(follower)
            }
        }
        base.configurator.apply(config, 0.050)

        val optimizeBusUtilization = optimizeUpdateRate == true
        if (optimizeBusUtilization) {
            base.optimizeBusUtilization()
            base.statorCurrent.setUpdateFrequency(50.0)
            base.motorVoltage.setUpdateFrequency(50.0)
        }
        for ((statusSignal, rate) in listOf(base.position to positionUpdateRate, base.velocity to velocityUpdateRate)){
            if (rate != null) {
                statusSignal.setUpdateFrequency(rate.inUnit(hertz))
            } else if (optimizeBusUtilization) {
                // if status frames have been optimized, setRate sets the default update rate
                // so that the frames don't get disabled
                statusSignal.setUpdateFrequency(50.0)
            }
        }

        return this
    }

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        require(positionPIDConfigured){" You must specify a positionPID value using the configure() method. "}
        setPosRequest.Position = position.inUnit(rotations)
        setPosRequest.FeedForward = feedforward.inUnit(volts)
        base.setControl(setPosRequest)
        runFollowing{ it.setPositionSetpoint(position, feedforward) }
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        require(velocityPIDConfigured){" You must specify a velocityPID value using the configure() method. "}
        setVelRequest.Velocity = velocity.inUnit(rotations / seconds)
        setVelRequest.FeedForward = feedforward.inUnit(volts)
        base.setControl(setVelRequest)
        runFollowing{ it.setVelocitySetpoint(velocity, feedforward) }
    }

    private val faultMap = mutableMapOf(
        base.fault_Hardware to "Hardware failure detected",
        base.fault_DeviceTemp to "Device temp exceeded limit",
        base.fault_BootDuringEnable to "Device booted when enabled",
        base.fault_FusedSensorOutOfSync to "Fused CANcoder out of sync",
        base.fault_OverSupplyV to "Voltage exceeded limit",
        base.fault_ProcTemp to "Processor is overheating",
        base.fault_Undervoltage to "Device supply voltage near brownout",
        base.fault_UnlicensedFeatureInUse to "Unlicensed feature in use(git Phoenix Pro Pls)",
        base.fault_UnstableSupplyV to "Supply voltage unstable",
    )

    override fun getFaults(deviceName: String): List<SubsystemFault> {
        val faults = mutableListOf<SubsystemFault>()
        for ((faultSignal, faultMsg) in faultMap){
            // != false prevents null
            if (faultSignal.refresh().value != false) faults.add(SubsystemFault("$deviceName: $faultMsg"))
        }
        if (voltageSignal.refresh().status != StatusCode.OK) {
            faults.add(SubsystemFault("$deviceName: Device is unreachable"))
        }
        return faults
    }
}
