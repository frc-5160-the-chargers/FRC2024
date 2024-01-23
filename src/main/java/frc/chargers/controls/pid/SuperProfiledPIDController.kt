@file:Suppress("MemberVisibilityCanBePrivate", "unused", "RedundantVisibilityModifier")
package frc.chargers.controls.pid

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.controls.motionprofiling.MotionProfile
import frc.chargers.controls.motionprofiling.MotionProfileState
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus
import kotlin.internal.LowPriorityInOverloadResolution


/**
 * Creates a [SuperProfiledPIDController] with a non-context function feedforward,
 * which accepts a Velocity(VI) and returns a Voltage.
 */
@LowPriorityInOverloadResolution
inline fun <PI: Dimension<*,*,*,*>,VI: Dimension<*,*,*,*>> SuperProfiledPIDController( // PI is the position input of the controller, while VI is the velocity input
    pidConstants: PIDConstants,
    motionProfile: MotionProfile<PI, VI>,
    crossinline feedforward: (Quantity<VI>) -> Voltage,
    noinline getInput: () -> Quantity<PI>,
    targetState: MotionProfileState<PI, VI>,
    continuousInputRange: ClosedRange<Quantity<PI>>? = null,
    outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    errorTolerance: Precision<PI> = Precision.AllowOvershoot,
    selfSustain: Boolean = false,
) = SuperProfiledPIDController(
    pidConstants,
    motionProfile,
    { feedforward(this.setpoint.velocity) },
    getInput, targetState,
    continuousInputRange, outputRange,
    errorTolerance, selfSustain
)

/**
 * Wraps WPILib's [edu.wpi.first.math.controller.PIDController], adding units support and support for a wide variety of motion profiles.
 *
 * Although this controller's basic functionality is similar to the [edu.wpi.first.math.controller.ProfiledPIDController],
 * it supports generic motion profiles(including exponential profiles) and built in feedforward.
 *
 * @see SuperPIDController for more details.
 */
class SuperProfiledPIDController<PosI: Dimension<*,*,*,*>, VelI: Dimension<*,*,*,*>>( // PosI is the position input of the controller; VelI is the velocity term, derived via the motion profile
    pidConstants: PIDConstants,
    public val motionProfile: MotionProfile<PosI, VelI>,
    // the feedforward function has the context of the controller, allowing for more flexibility.
    private val feedforward: SuperProfiledPIDController<PosI, VelI>.() -> Voltage = { Voltage(0.0) },
    private val getInput: () -> Quantity<PosI>,
    /**
     * The target state of the PID controller; represented as a [MotionProfileState].
     */
    var targetState: MotionProfileState<PosI, VelI>,
    continuousInputRange: ClosedRange<Quantity<PosI>>? = null,
    outputRange: ClosedRange<Voltage> = Voltage(Double.NEGATIVE_INFINITY)..Voltage(Double.POSITIVE_INFINITY),
    errorTolerance: Precision<PosI> = Precision.AllowOvershoot,
    selfSustain: Boolean = true
    // profiled PID controller is voltage only!
) : SuperPIDController<PosI, VoltageDimension>(
    pidConstants,
    // feedforward not passed into the base controller due to it being separately calculated
    getInput = getInput,
    target = getInput(), // keeps the target at 0 for now; will be modified via calculateOutput()
    continuousInputRange = continuousInputRange,
    outputRange = outputRange,
    errorTolerance = errorTolerance,
    selfSustain = selfSustain
){
    /**
     * The setpoint of the PID controller. This is not the goal; instead, it is the current setpoint
     * that the underlying pid controller wants to follow.
     *
     * The goal of a profiled pid controller is to follow multiple of these setpoints to obtain smoother motion
     */
    var setpoint = MotionProfileState<PosI,VelI>(getInput())
        private set

    override var target: Quantity<PosI>
        get() = targetState.position
        set(value) {
            targetState = MotionProfileState(value)
        }

    override fun calculateOutput(): Voltage {
        val input = getInput()
        // optimizes continuous input for motion profiles
        if (continuousInputRange != null){
            val errorBound = (continuousInputRange.endInclusive - continuousInputRange.start) / 2.0
            val goalMinDistance =
                (targetState.position - input)
                    .inputModulus(-errorBound..errorBound)
            val setpointMinDistance =
                (setpoint.position - input)
                    .inputModulus(-errorBound..errorBound)
            targetState.position = goalMinDistance + input
            setpoint.position = setpointMinDistance + input
        }

        // steps the setpoint by 0.02 seconds closer to the goal
        setpoint = motionProfile.calculate(pidController.period.ofUnit(seconds), setpoint, targetState)

        super.target = setpoint.position
        return super.calculateOutput() + feedforward()
    }
}