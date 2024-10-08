package frc.chargers.hardware.motorcontrol

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import com.revrobotics.*
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import edu.wpi.first.wpilibj.DriverStation
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
    private val faultLogName: String? = null
): Motor {
    val deviceID: Int = base.deviceId

    private val nonRevFollowers = mutableListOf<Motor>()
    private val relativeEncoder = base.getEncoder()
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
            get() = relativeEncoder.velocity.ofUnit(rotations / minutes)
        override val angularPosition: Angle
            get() = relativeEncoder.position.ofUnit(rotations)
    }
    private inner class AbsoluteEncoderImpl: Encoder {
        override val angularVelocity: AngularVelocity
            get() = absoluteEncoder.velocity.ofUnit(rotations / seconds)
        override val angularPosition: Angle
            get() = absoluteEncoder.position.ofUnit(rotations)
    }

    override var voltageOut: Voltage
        get() = Voltage(base.appliedOutput * base.busVoltage)
        set(voltage){
            base.setVoltage(voltage.inUnit(volts))
        }

    override val statorCurrent: Current get() = Current(base.outputCurrent)

    override val inverted: Boolean get() = base.inverted

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        if (!positionPIDConfigured) {
            DriverStation.reportError("You must specify a positionPID value using the " +
                    "motor.configure(positionPID = PIDConstants(p,i,d)) method.", true)
            return
        } else if (abs(position - this.encoder.angularPosition) < 1.degrees) {
            base.setVoltage(0.0)
            return
        }
        base.pidController.setReference(
            position.inUnit(rotations),
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.inUnit(volts)
        )
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        if (!velocityPIDConfigured) {
            DriverStation.reportError("You must specify a positionPID value using the " +
                    "motor.configure(velocityPID = PIDConstants(p,i,d)) method.", true)
            return
        }
        base.pidController.setReference(
            if (useAbsoluteEncoder) {
                velocity.inUnit(rotations / seconds)
            } else {
                velocity.inUnit(rotations / minutes)
            },
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
        val errors = mutableListOf<REVLibError>()
        fun REVLibError.bind() { if (this != REVLibError.kOk) errors.add(this) }
        for (i in 1..4) {
            if (inverted != null) base.inverted = inverted
            when (brakeWhenIdle) {
                true -> base.setIdleMode(CANSparkBase.IdleMode.kBrake).bind()
                false -> base.setIdleMode(CANSparkBase.IdleMode.kCoast).bind()
                null -> {}
            }
            if (rampRate != null) {
                base.setOpenLoopRampRate(rampRate.inUnit(seconds)).bind()
                base.setClosedLoopRampRate(rampRate.inUnit(seconds)).bind()
            }
            if (statorCurrentLimit != null) base.setSmartCurrentLimit(statorCurrent.inUnit(amps).toInt()).bind()
            for (follower in followerMotors) {
                follower.configure(
                    positionPID = positionPID,
                    velocityPID = velocityPID,
                    gearRatio = gearRatio,
                    startingPosition = startingPosition
                )
                when (follower) {
                    is ChargerSpark<*> -> follower.base.follow(base, follower.inverted).bind()
                    else -> nonRevFollowers.add(follower)
                }
            }
            if (gearRatio != null && !useAbsoluteEncoder) {
                relativeEncoder.setPositionConversionFactor(1 / gearRatio).bind()
                relativeEncoder.setVelocityConversionFactor(1 / gearRatio).bind()
            }
            if (startingPosition != null) relativeEncoder.setPosition(startingPosition.inUnit(rotations)).bind()
            // 2 * PI makes it so that the PID gains are optimized off of radians and not rotations
            if (positionPID != null) {
                positionPIDConfigured = true
                base.pidController.apply {
                    setP(positionPID.kP * (2 * PI), 0).bind()
                    setI(positionPID.kI * (2 * PI), 0).bind()
                    setD(positionPID.kD * (2 * PI), 0).bind()
                }
            }
            if (velocityPID != null) {
                velocityPIDConfigured = true
                // relative encoder pid is based off of rotations/min so we have to compensate here
                val multiplier = 2 * PI * if (useAbsoluteEncoder) 1.0 else 60.0
                base.pidController.apply {
                    setP(velocityPID.kP * multiplier, 1).bind()
                    setI(velocityPID.kI * multiplier, 1).bind()
                    setD(velocityPID.kD * multiplier, 1).bind()
                }
            }
            if (continuousInput == true) {
                base.pidController.apply {
                    setPositionPIDWrappingEnabled(true).bind()
                    setPositionPIDWrappingMinInput(-180.degrees.inUnit(rotations)).bind()
                    setPositionPIDWrappingMaxInput(180.degrees.inUnit(rotations)).bind()
                }
            } else if (continuousInput == false) {
                base.pidController.setPositionPIDWrappingEnabled(false).bind()
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
                ).bind()
            }
            if (velocityUpdateRate != null) {
                base.setPeriodicFramePeriod(
                    if (useAbsoluteEncoder) PeriodicFrame.kStatus6 else PeriodicFrame.kStatus1,
                    frequencyToPeriod(velocityUpdateRate).inUnit(milli.seconds).toInt()
                ).bind()
            }
            base.burnFlash().bind()
            if (errors.isEmpty()) return this
            errors.clear()
        }
        DriverStation.reportError("ERROR: ${faultLogName ?: "ChargerSpark($deviceID)"} could not configure. Errors: $errors", false)
        return this
    }
}
