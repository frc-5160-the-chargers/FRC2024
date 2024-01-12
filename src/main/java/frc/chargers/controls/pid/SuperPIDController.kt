@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.SetpointSupplier
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus

/**
 * Wraps WPILib's [PIDController], adding various improvements and units support.
 *
 * This class accepts Kmeasure [Quantity]s instead of [Double]s:
 * converting them to their siValue before and after the pid equation is applied.
 *
 * This ensures that the pid constants passed in do not need to change for different units;
 * they should be tuned with the respect to the SI unit of the input and output.
 *
 * See [here](https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html) for an explanation of PID.
 */
public class SuperPIDController<I: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>>(
    /**
     * The PID constants of the controller.
     */
    pidConstants: PIDConstants,
    /**
     * A function that fetches the controller input.
     */
    private val getInput: () -> Quantity<I>,
    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    override var target: Quantity<I>,
    /**
     * The setpoint supplier converts a target into one or multiple setpoints
     * for the pid controller to run to.
     *
     * This can include trapezoidal or exponential motion profiles.
     */
    private val setpointSupplier: SetpointSupplier<I, O> = SetpointSupplier.Default(),
    /**
     * If this value is not null, enables continuous input within a certain [ClosedRange],
     * and wraps all target values/input values within the range.
     *
     * see [here](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html#setting-continuous-input)
     * for an explanation on continuous input.
     */
    private val continuousInputRange: ClosedRange<Quantity<I>>? = null,
    /**
     * Optionally clamps the output of the controller within a certain [ClosedRange].
     */
    private val outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
    /**
     * Controls the error tolerance of the controller.
     */
    errorTolerance: Precision<I> = Precision.AllowOvershoot,
    /**
     * Determines if the [SuperPIDController] should call calculateOutput()
     * during every loop of the command scheduler. Normal PID controllers require the user to do this.
     */
    selfSustain: Boolean = false,
) : FeedbackController<Quantity<I>, Quantity<O>> {
    private fun Quantity<I>.standardize(): Quantity<I> =
        if (continuousInputRange == null) this else this.inputModulus(continuousInputRange)


    private val pidController = PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD)

    init{
        if (selfSustain){
            ChargerRobot.runPeriodically(runnable = ::calculateOutput)
        }

        if (continuousInputRange != null){
            pidController.enableContinuousInput(
                continuousInputRange.start.siValue,
                continuousInputRange.endInclusive.siValue
            )
        }

        if (errorTolerance is Precision.Within){
            pidController.setTolerance(
                (errorTolerance.allowableError.endInclusive - errorTolerance.allowableError.start).siValue
            )
        }
    }


    /**
     * Calculates the output of the PID controller, using the calculated error.
     */
    override fun calculateOutput(): Quantity<O> {
        val input = getInput()
        val setpoint = if (continuousInputRange == null){
            setpointSupplier.calculateSetpoint(target)
        }else{
            setpointSupplier.calculateSetpoint(
                target.standardize(),
                continuousInputRange,
                input.standardize()
            )
        }
        val pidOutput = Quantity<O>(
            pidController.calculate(
                input.standardize().siValue,
                setpoint.value.standardize().siValue
            )
        )
        val ffOutput = setpoint.feedforwardOutput
        return (pidOutput + ffOutput).coerceIn(outputRange)
    }


    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     */
    override val error: Quantity<I>
        get() = Quantity(pidController.positionError)


    /**
     * A variable that holds the [PIDConstants] of the controller.
     */
    public var constants: PIDConstants
        get() = PIDConstants(pidController.p, pidController.i, pidController.d)
        set(value){
            if (value != constants){
                pidController.reset()
                pidController.p = value.kP
                pidController.i = value.kI
                pidController.d = value.kD
            }
        }

}