@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import com.revrobotics.REVLibError
import frc.chargers.hardware.configuration.HardwareConfiguration
import kotlin.math.roundToInt

/**
 * Represents data that can flow into a spark device.
 */
public enum class MotorData{
    POSITION, VELOCITY, VOLTAGE, TEMPERATURE, CURRENT
}

/**
 * Represents status frame configuration for a Spark device; this includes the
 * ChargerSparkMax and ChargerSparkFlex.
 *
 * @see PeriodicFrameConfig.Optimized
 * @see PeriodicFrameConfig.Custom
 */
public sealed class PeriodicFrameConfig{
    /**
     * Represents an optimized status frame configuration, with values for
     * kStatus1, kStatus2, kStatus3, kStatus4, etc.
     */
    public data class Optimized(
        /**
         * Determines whether to optimize encoder frames.
         *
         * Set this property to FALSE
         * if you are manually using getEncoder(), getAbsoluteEncoder(), etc.
         */
        val optimizeEncoderFrames: Boolean = true,
        val utilizedData: List<MotorData> = listOf(MotorData.POSITION, MotorData.VELOCITY, MotorData.VOLTAGE, MotorData.TEMPERATURE, MotorData.CURRENT)
    ): PeriodicFrameConfig()

    /**
     * Represents a custom status frame configuration, with values for
     * kStatus1, kStatus2, kStatus3, kStatus4, etc.
     */
    public data class Custom(
        val frames: MutableMap<PeriodicFrame, Int>
    ): PeriodicFrameConfig() {
        public constructor(
            kStatus0: Int? = null,
            kStatus1: Int? = null,
            kStatus2: Int? = null,
            kStatus3: Int? = null,
            kStatus4: Int? = null,
            kStatus5: Int? = null,
            kStatus6: Int? = null
        ) : this(
            mutableMapOf<PeriodicFrame, Int>().apply {
                if (kStatus0 != null) put(PeriodicFrame.kStatus0, kStatus0)
                if (kStatus1 != null) put(PeriodicFrame.kStatus1, kStatus1)
                if (kStatus2 != null) put(PeriodicFrame.kStatus2, kStatus2)
                if (kStatus3 != null) put(PeriodicFrame.kStatus3, kStatus3)
                if (kStatus4 != null) put(PeriodicFrame.kStatus4, kStatus4)
                if (kStatus5 != null) put(PeriodicFrame.kStatus5, kStatus5)
                if (kStatus6 != null) put(PeriodicFrame.kStatus6, kStatus6)
            }
        )
    }
}


/**
 * Represents a smart current limit for REV motors.
 */
public data class SmartCurrentLimit(
    val stallLimit: Current,
    val freeLimit: Current? = null,
    val limitSpeed: AngularVelocity? = null
)

/**
 * Represents a secondary current limit for REV motors.
 */
public data class SecondaryCurrentLimit(
    val limit: Current,
    val chopCycles: Int? = null
)


/**
 * A base class that represents configuration for REV motors;
 * these include the ChargerCANSparkMax and the ChargerCANSparkFlex.
 *
 * These configurations have a 1-to-1 correspondence with the "set" functions of [CANSparkBase];
 * however, Kmeasure is used whenever possible.
 */
@Suppress("MemberVisibilityCanBePrivate")
public class ChargerSparkConfiguration(
    public var encoderType: SparkEncoderType? = null,
    public var idleMode: CANSparkBase.IdleMode? = null,
    public var inverted: Boolean? = null,
    public var voltageCompensationNominalVoltage: Voltage? = null,
    public var canTimeout: Time? = null,
    public var closedLoopRampRate: Double? = null,
    public var openLoopRampRate: Double? = null,
    public var controlFramePeriod: Time? = null,
    public var periodicFrameConfig: PeriodicFrameConfig? = null,
    public var smartCurrentLimit: SmartCurrentLimit? = null,
    public var secondaryCurrentLimit: SecondaryCurrentLimit? = null,
    public var softLimits: MutableMap<CANSparkBase.SoftLimitDirection, Angle> = mutableMapOf(),
): HardwareConfiguration {

    public fun setSoftLimit(direction: CANSparkBase.SoftLimitDirection, limit: Angle){
        softLimits[direction] = limit
    }

    public fun setSmartCurrentLimit(limit: Current){
        smartCurrentLimit = SmartCurrentLimit(limit)
    }

    public fun setSmartCurrentLimit(
        stallLimit: Current,
        freeLimit: Current,
        limitSpeed: AngularVelocity? = null
    ){
        smartCurrentLimit = SmartCurrentLimit(stallLimit, freeLimit, limitSpeed)
    }

    public fun setSecondaryCurrentLimit(limit: Current, chopCycles: Int? = null){
        secondaryCurrentLimit = SecondaryCurrentLimit(limit, chopCycles)
    }


    internal fun applyTo(motor: CANSparkBase): List<REVLibError>{
        val allErrors: MutableList<REVLibError> = mutableListOf()

        fun REVLibError?.updateConfigStatus(): REVLibError? {
            if (this != null && this != REVLibError.kOk) {
                allErrors.add(this)
            }
            return this
        }

        // ?.let only calls the function(with it as the receiver)
        // if the configuration is not null.
        // thus, it also returns a RevlibError, which will be processed
        // by the motor to determine if all configurations have gone through or not.
        idleMode?.let(motor::setIdleMode).updateConfigStatus()
        inverted?.let(motor::setInverted)
        voltageCompensationNominalVoltage?.let { motor.enableVoltageCompensation(it.inUnit(volts)) }.updateConfigStatus()
        canTimeout?.let { timeout -> motor.setCANTimeout(timeout.inUnit(milli.seconds).roundToInt()) }.updateConfigStatus()
        closedLoopRampRate?.let(motor::setClosedLoopRampRate).updateConfigStatus()
        openLoopRampRate?.let(motor::setOpenLoopRampRate).updateConfigStatus()
        controlFramePeriod?.let { period -> motor.setControlFramePeriodMs(period.inUnit(milli.seconds).roundToInt()) }
        smartCurrentLimit?.let { (stallLimit, freeLimit, limitSpeed) ->
            when {
                limitSpeed != null && freeLimit != null ->
                    motor.setSmartCurrentLimit(
                        stallLimit.inUnit(amps).roundToInt(),
                        freeLimit.inUnit(amps).roundToInt(),
                        limitSpeed.inUnit(rotations / minutes).roundToInt()
                    )
                freeLimit != null -> motor.setSmartCurrentLimit(
                    stallLimit.inUnit(amps).roundToInt(),
                    freeLimit.inUnit(amps).roundToInt()
                )
                else -> motor.setSmartCurrentLimit(
                    stallLimit.inUnit(amps).roundToInt()
                )
            }
        }.updateConfigStatus()
        secondaryCurrentLimit?.let { (limit, chopCycles) ->
            when {
                chopCycles != null -> motor.setSecondaryCurrentLimit(limit.inUnit(amperes), chopCycles)
                else -> motor.setSecondaryCurrentLimit(limit.inUnit(amperes))
            }
        }.updateConfigStatus()
        for ((limitDirection, limit) in softLimits) {
            motor.setSoftLimit(limitDirection, limit.inUnit(rotations).toFloat()).updateConfigStatus()
        }
        
        when (val frameConfig = periodicFrameConfig){
            is PeriodicFrameConfig.Custom -> {
                frameConfig.frames.forEach{ (frame, period) ->
                    motor.setPeriodicFramePeriod(frame, period).updateConfigStatus()
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
                    MotorData.VOLTAGE in frameConfig.utilizedData ||
                    MotorData.CURRENT in frameConfig.utilizedData
                ) {
                    status1 = FAST_PERIODIC_FRAME_STRATEGY
                }

                if (MotorData.POSITION in frameConfig.utilizedData) {
                    status2 = FAST_PERIODIC_FRAME_STRATEGY
                }

                if (frameConfig.optimizeEncoderFrames){
                    println(this.encoderType)
                    when (this.encoderType){
                        is SparkEncoderType.DutyCycle -> {
                            status4 = FAST_PERIODIC_FRAME_STRATEGY
                        }

                        is SparkEncoderType.Quadrature -> {
                            if (MotorData.POSITION in frameConfig.utilizedData){
                                status5 = FAST_PERIODIC_FRAME_STRATEGY
                            }
                            if (MotorData.VELOCITY in frameConfig.utilizedData){
                                status6 = FAST_PERIODIC_FRAME_STRATEGY
                            }
                        }

                        is SparkEncoderType.Regular, null -> {}
                    }
                    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1).updateConfigStatus()
                    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2).updateConfigStatus()
                    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4).updateConfigStatus()
                    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5).updateConfigStatus()
                    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6).updateConfigStatus()
                }
            }

            null -> {}
        }
        
        return allErrors
    }
}