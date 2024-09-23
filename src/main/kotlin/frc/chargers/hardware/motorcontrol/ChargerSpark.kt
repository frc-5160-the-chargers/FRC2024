package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import com.revrobotics.*
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog
import frc.chargers.hardware.sensors.encoders.Encoder
import frc.chargers.utils.units.frequencyToPeriod
import kotlin.math.PI


/**
 * Creates a [CANSparkMax] that implements the [Motor] interface.
 *
 * @see ChargerSpark
 */
class ChargerSparkMax(
    deviceID: Int,
    motorType: CANSparkLowLevel.MotorType = CANSparkLowLevel.MotorType.kBrushless,
    useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true,
    faultLogName: String? = null
): ChargerSpark<CANSparkMax>(
    CANSparkMax(deviceID, motorType),
    useAbsoluteEncoder, factoryDefault, faultLogName
)


/**
 * Creates a [CANSparkFlex] that implements the [Motor] interface.
 *
 * @see ChargerSpark
 */
class ChargerSparkFlex(
    deviceID: Int,
    useAbsoluteEncoder: Boolean = false,
    factoryDefault: Boolean = true,
    faultLogName: String? = null
): ChargerSpark<CANSparkFlex>(
    CANSparkFlex(deviceID, CANSparkLowLevel.MotorType.kBrushless),
    useAbsoluteEncoder, factoryDefault, faultLogName
)


/**
 * A utility class that implements the [Motor] interface
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
    factoryDefault: Boolean = true,
    faultLogName: String? = null
): Motor {
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

        if (faultLogName != null) {
            ChargerRobot.runPeriodicAtPeriod(0.1.seconds) {
                val err = base.lastError
                if (err != REVLibError.kOk){
                    HorseLog.logFault("$faultLogName: $err")
                }
            }
        }
    }

    override val encoder: Encoder = if (useAbsoluteEncoder) AbsoluteEncoderImpl() else RelativeEncoderImpl()
    private inner class RelativeEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity
            get() = relativeEncoder.velocity.ofUnit(rotations / seconds)
        override val angularPosition: Angle
            get() = relativeEncoder.position.ofUnit(rotations)
    }
    private inner class AbsoluteEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity
            get() = absoluteEncoder.velocity.ofUnit(rotations / minutes)
        override val angularPosition: Angle
            get() = absoluteEncoder.position.ofUnit(rotations)
    }

    override var appliedVoltage: Voltage
        get() = Voltage(base.appliedOutput * base.busVoltage)
        set(voltage){
            base.setVoltage(voltage.inUnit(volts))
        }

    override val statorCurrent: Current get() = Current(base.outputCurrent)

    override val inverted: Boolean get() = base.inverted

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        require(positionPIDConfigured){" You must specify a positionPID value using the configure(positionPID = PIDConstants(p,i,d)) method. "}
        base.pidController.setReference(
            position.inUnit(rotations),
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.inUnit(volts)
        )
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        require(velocityPIDConfigured){" You must specify a velocityPID value using the configure(velocityPID = PIDConstants(p,i,d)) method. "}
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
        // 2 * PI makes it so that the PID gains are optimized off of radians and not rotations
        if (positionPID != null) {
            positionPIDConfigured = true
            base.pidController.apply {
                setP(positionPID.kP * (2 * PI), 0)
                setI(positionPID.kI * (2 * PI), 0)
                setD(positionPID.kD * (2 * PI), 0)
            }
        }
        if (velocityPID != null) {
            velocityPIDConfigured = true
            // ensures that PID units are rotations / second
            val multiplier = 2 * PI * if (useAbsoluteEncoder) 60.0 else 1.0
            base.pidController.apply {
                setP(velocityPID.kP * multiplier, 1)
                setI(velocityPID.kI * multiplier, 1)
                setD(velocityPID.kD * multiplier, 1)
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
}
