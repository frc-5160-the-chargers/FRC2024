package frc.chargers.hardware.motorcontrol.rev

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import com.revrobotics.REVLibError
import com.revrobotics.SparkAbsoluteEncoder
import frc.chargers.framework.faultchecking.FaultChecking
import frc.chargers.framework.faultchecking.SubsystemFault
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.utils.filterNaN
import frc.chargers.utils.math.units.frequencyToPeriod
import frc.chargers.utils.withSetter


/**
 * A utility class that implements the [Motor] and [FaultChecking] interfaces
 * for spark max/flex motors.
 *
 * To access the base motor, use the [base] property.
 */
open class ChargerSpark<BaseMotorType: CANSparkBase>(
    /**
     * The base Spark max/flex instance.
     */
    val base: BaseMotorType,
    private val useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true
): Motor, FaultChecking {
    val deviceID: Int = base.deviceId

    private val nonRevFollowers = mutableListOf<Motor>()
    private val relativeEncoder = base.encoder
    private val absoluteEncoder = base.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private var positionPIDConfigured = false
    private var velocityPIDConfigured = false

    init {
        if (useAbsoluteEncoder) {
            base.pidController.setFeedbackDevice(base.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle))
        }
        if (factoryDefault) {
            base.restoreFactoryDefaults()
        }
        base.enableVoltageCompensation(12.0)
        base.burnFlash()
    }

    override val encoder: Encoder = if (useAbsoluteEncoder) AbsoluteEncoderImpl() else RelativeEncoderImpl()
    private inner class RelativeEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity by filterNaN { relativeEncoder.velocity.ofUnit(rotations / seconds) }
        override val angularPosition: Angle by filterNaN { relativeEncoder.position.ofUnit(rotations) }
    }
    private inner class AbsoluteEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity by filterNaN { absoluteEncoder.velocity.ofUnit(rotations / minutes) }
        override val angularPosition: Angle by filterNaN { absoluteEncoder.position.ofUnit(rotations) }
    }

    override var appliedVoltage: Voltage
        by filterNaN { Voltage(base.appliedOutput * base.busVoltage) }
            .withSetter { base.setVoltage(it.siValue) }

    override val statorCurrent: Current by filterNaN { Current(base.outputCurrent) }

    override val inverted: Boolean get() = base.inverted

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        require(positionPIDConfigured){" You must specify a positionPID value using the configure() method. "}
        base.pidController.setReference(
            position.inUnit(rotations),
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.inUnit(volts)
        )
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        require(velocityPIDConfigured){" You must specify a velocityPID value using the configure() method. "}
        val velocityAsDouble = if (useAbsoluteEncoder) {
            velocity.inUnit(rotations / minutes)
        } else {
            velocity.inUnit(rotations / seconds)
        }
        base.pidController.setReference(
            velocityAsDouble,
            CANSparkBase.ControlType.kVelocity,
            1,
            feedforward.inUnit(volts)
        )
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
    ): ChargerSpark<BaseMotorType> {
        if (inverted != null) base.inverted = inverted
        if (brakeWhenIdle == true) {
            base.idleMode = CANSparkBase.IdleMode.kBrake
        } else if (brakeWhenIdle == false) {
            base.idleMode = CANSparkBase.IdleMode.kCoast
        }
        if (rampRate != null) {
            base.openLoopRampRate = rampRate.inUnit(seconds)
            base.closedLoopRampRate = rampRate.inUnit(seconds)
        }
        if (statorCurrentLimit != null) base.setSmartCurrentLimit(statorCurrent.inUnit(amps).toInt())
        for (follower in followerMotors) {
            follower.configure(
                positionPID = positionPID,
                velocityPID = velocityPID,
                gearRatio = gearRatio,
                startingPosition = startingPosition
            )
            when (follower) {
                is ChargerSpark<*> -> follower.base.follow(base, follower.inverted)
                else -> nonRevFollowers.add(follower)
            }
        }
        if (gearRatio != null && !useAbsoluteEncoder) {
            relativeEncoder.positionConversionFactor = 1 / gearRatio
            relativeEncoder.velocityConversionFactor = 1 / gearRatio
        }
        if (startingPosition != null) relativeEncoder.setPosition(startingPosition.inUnit(rotations))
        if (positionPID != null) {
            positionPIDConfigured = true
            base.pidController.apply {
                setP(positionPID.kP, 0)
                setI(positionPID.kI, 0)
                setD(positionPID.kD, 0)
            }
        }
        if (velocityPID != null) {
            velocityPIDConfigured = true
            base.pidController.apply {
                setP(velocityPID.kP, 1)
                setI(velocityPID.kI, 1)
                setD(velocityPID.kD, 1)
            }
        }
        if (continuousInput == true) {
            base.pidController.apply {
                positionPIDWrappingEnabled = true
                positionPIDWrappingMinInput = -180.degrees.inUnit(rotations)
                positionPIDWrappingMaxInput = 180.degrees.inUnit(rotations)
            }
        } else if (continuousInput == false) {
            base.pidController.positionPIDWrappingEnabled = false
        }
        if (optimizeUpdateRate == true) {
            val disabledFrames = if (useAbsoluteEncoder) {
                listOf(PeriodicFrame.kStatus2, PeriodicFrame.kStatus3, PeriodicFrame.kStatus4)
            } else {
                listOf(PeriodicFrame.kStatus3, PeriodicFrame.kStatus4, PeriodicFrame.kStatus5, PeriodicFrame.kStatus6)
            }
            for (frame in disabledFrames) {
                base.setPeriodicFramePeriod(frame, 65535)
            }
        }
        if (positionUpdateRate != null) {
            base.setPeriodicFramePeriod(
                if (useAbsoluteEncoder) PeriodicFrame.kStatus5 else PeriodicFrame.kStatus2,
                frequencyToPeriod(positionUpdateRate).inUnit(milli.seconds).toInt()
            )
        }
        if (velocityUpdateRate != null) {
            base.setPeriodicFramePeriod(
                if (useAbsoluteEncoder) PeriodicFrame.kStatus6 else PeriodicFrame.kStatus1,
                frequencyToPeriod(velocityUpdateRate).inUnit(milli.seconds).toInt()
            )
        }

        base.burnFlash()
        return this
    }

    override fun getFaults(deviceName: String): List<SubsystemFault> {
        val err: REVLibError = base.lastError
        return if (err != REVLibError.kOk) {
            listOf(SubsystemFault("$deviceName: Error: ${err.name}"))
        } else {
            emptyList()
        }
    }
}
