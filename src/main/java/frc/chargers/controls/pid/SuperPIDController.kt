@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.FeedbackController
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus
import kotlin.internal.LowPriorityInOverloadResolution


@LowPriorityInOverloadResolution
inline fun <I: Dimension<*,*,*,*>,O: Dimension<*,*,*,*>> SuperPIDController(
    pidConstants: PIDConstants,
    crossinline feedforward: (Quantity<I>) -> Quantity<O>,
    noinline getInput: () -> Quantity<I>,
    target: Quantity<I>,
    continuousInputRange: ClosedRange<Quantity<I>>? = null,
    outputRange: ClosedRange<Quantity<O>> = Quantity<O>(Double.NEGATIVE_INFINITY)..Quantity(Double.POSITIVE_INFINITY),
    errorTolerance: Precision<I> = Precision.AllowOvershoot,
    selfSustain: Boolean = false,
) = SuperPIDController(
    pidConstants,
    { feedforward(this.target) },
    getInput, target,
    continuousInputRange, outputRange,
    errorTolerance, selfSustain
)





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
public open class SuperPIDController<I: Dimension<*,*,*,*>, O: Dimension<*,*,*,*>>(
    /**
     * The PID constants of the controller.
     */
    pidConstants: PIDConstants,
    /**
     * a Lambda function that fetches the feedforward of the controller.
     *
     * Has the context of this class; which means that it can access properties directly(such as target, setpoint, etc.)
     */
    private val feedforward: SuperPIDController<I,O>.() -> Quantity<O> = { Quantity(0.0) },
    /**
     * A function that fetches the controller input.
     */
    private val getInput: () -> Quantity<I>,
    /**
     * The target is the value the PID controller is attempting to achieve.
     */
    target: Quantity<I>,
    /**
     * If this value is not null, enables continuous input within a certain [ClosedRange],
     * and wraps all target values/input values within the range.
     *
     * see [here](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html#setting-continuous-input)
     * for an explanation on continuous input.
     */
    protected val continuousInputRange: ClosedRange<Quantity<I>>? = null,
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

    protected val pidController = PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD)

    protected open fun resetController(){
        pidController.reset()
    }

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

        pidController.setpoint = target.siValue
    }


    /**
     * Calculates the output of the PID controller, using the calculated error.
     */
    override fun calculateOutput(): Quantity<O> {
        val pidOutput = Quantity<O>(
            pidController.calculate(
                if (continuousInputRange == null){
                    getInput().siValue
                }else {
                    getInput().inputModulus(continuousInputRange).siValue
                }
            )
        )
        return (pidOutput + feedforward()).coerceIn(outputRange)
    }


    /**
     * The error is a signed value representing how far the PID system currently is from the target value.
     * Note: this error is not the pidController's traditional getPositionError() function;
     * this is because that function wraps the error within the pid controller's continuous input,
     * which is not desirable here.
     */
    override val error: Quantity<I>
        get() = Quantity<I>(pidController.setpoint) - getInput()

    /**
     * Determines if the controller is at it's setpoint.
     */
    public val atSetpoint: Boolean
        get() = pidController.atSetpoint()


    override var target: Quantity<I>
        get() = Quantity(pidController.setpoint)
        set(value){
            pidController.setpoint = value.siValue
        }


    /**
     * A variable that holds the [PIDConstants] of the controller.
     */
    public var constants: PIDConstants
        get() = PIDConstants(pidController.p, pidController.i, pidController.d)
        set(value){
            if (value != constants){
                resetController()
                pidController.setPID(value.kP, value.kI, value.kD)
            }
        }

    /**
     * Disables self sustain for controllers.
     */
    public fun disableSelfSustain(){
        ChargerRobot.removeFromLoop(::calculateOutput)
    }
}