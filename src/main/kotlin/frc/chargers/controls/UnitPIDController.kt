@file:Suppress("unused")
package frc.chargers.controls

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import frc.chargers.framework.ChargerRobot

/**
 * A wrapper for WPILib's [PIDController], adding in support for units.
 *
 * The UnitPIDController can be called in the exact same way as a wpilib pid controller can;
 * however, it adds support for optional output clamping.
 *
 * Note: This controller does not support IZone(no one uses it lol).
 */
class UnitPIDController<I: AnyDimension, O: AnyDimension>(
    kP: Number,
    kI: Number,
    kD: Number,
    continuousInputRange: ClosedRange<Quantity<I>>? = null,
    var outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
) {
    constructor(
        constants: PIDConstants,
        continuousInputRange: ClosedRange<Quantity<I>>? = null,
        outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY)
    ): this(constants.kP, constants.kI, constants.kD, continuousInputRange, outputRange)


    private val pidController = PIDController(kP.toDouble(), kI.toDouble(), kD.toDouble(), ChargerRobot.LOOP_PERIOD.inUnit(seconds))

    init{
        if (continuousInputRange != null){
            enableContinuousInput(continuousInputRange)
        }
    }

    /**
     * Calculates the output of the [UnitPIDController].
     *
     * You only need to specify the [feedforwardOutput] argument if you are using built-in output clamping
     * and have a complementary feedforward; otherwise, leave it empty.
     */
    fun calculate(
        measurement: Quantity<I>,
        setpoint: Quantity<I>,
        feedforwardOutput: Quantity<O> = Quantity(0.0)
    ): Quantity<O> =
        (Quantity<O>(pidController.calculate(measurement.siValue, setpoint.siValue)) + feedforwardOutput)
            .coerceIn(outputRange)

    fun enableContinuousInput(range: ClosedRange<Quantity<I>>){
        pidController.enableContinuousInput(
            range.start.siValue,
            range.endInclusive.siValue
        )
    }

    fun enableContinuousInput(start: Quantity<I>, end: Quantity<I>){
        enableContinuousInput(start..end)
    }

    fun disableContinuousInput(){
        pidController.disableContinuousInput()
    }

    fun setPID(kP: Number, kI: Number, kD: Number){
        setPID(kP.toDouble(), kI.toDouble(), kD.toDouble())
    }

    val positionError: Quantity<I>
        get() = Quantity(pidController.positionError)

    val velocityError: Quantity<I>
        get() = Quantity(pidController.velocityError)

    val atSetpoint: Boolean
        get() = pidController.atSetpoint()

    val isContinuousInputEnabled: Boolean
        get() = pidController.isContinuousInputEnabled

    var constants: PIDConstants
        get() = PIDConstants(pidController.p, pidController.i, pidController.d)
        set(value){
            pidController.setPID(value.kP, value.kI, value.kD)
        }
}