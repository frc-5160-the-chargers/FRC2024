@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import frc.chargers.controls.Setpoint
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.wpilibextensions.fpgaTimestamp

/**
 * A [SetpointSupplier] instance which derives Angular [Setpoint]s
 * (consisting of the setpoint(Angle) and a feedforward value(Voltage))
 * from an [ExponentialProfile].
 */
public class AngularExponentialSetpointSupplier(
    profileConstraints: Constraints,
    /**
     * The feedforward helps the mechanism reach the velocity output of the motion profile.
     */
    private val velocityTargetFF: Feedforward<AngularVelocityDimension, VoltageDimension>,
    startingState: State = State(0.0,0.0)
): ExponentialProfile(profileConstraints), SetpointSupplier<AngleDimension, VoltageDimension>{

    private var currentState = startingState
    private var previousT = fpgaTimestamp()


    /**
     * Fetches the current position that the [SetpointSupplier]
     * wants to direct the mechanism to.
     */
    public val setpointPosition: Angle
        get() = Angle(currentState.position)

    /**
     * Fetches the current velocity that the [SetpointSupplier]
     * wants to direct the mechanism to.
     */
    public val setpointVelocity: AngularVelocity
        get() = AngularVelocity(currentState.velocity)


    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile and a target position.
     */
    override fun calculateSetpoint(target: Angle): Setpoint<AngleDimension, VoltageDimension> =
        getSetpoint(target, AngularVelocity(0.0))


    override fun calculateSetpoint(
        target: Angle,
        continuousInputRange: ClosedRange<Angle>,
        measurement: Angle
    ): Setpoint<AngleDimension, VoltageDimension> =
        calculateSetpoint(target, AngularVelocity(0.0), continuousInputRange, measurement)


    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity,
     * with a specified continuous input range.
     */
    public fun calculateSetpoint(
        targetPosition: Angle,
        targetVelocity: AngularVelocity,
        continuousInputRange: ClosedRange<Angle>,
        measurement: Angle
    ): Setpoint<AngleDimension, VoltageDimension>{
        var goalState = State(targetPosition.siValue, targetVelocity.siValue)
        val setpointGoalPair = optimizeMotionProfileTargets(
            currentState, goalState,
            continuousInputRange.start.siValue..continuousInputRange.endInclusive.siValue,
            measurement.siValue
        )
        currentState = setpointGoalPair.first
        goalState = setpointGoalPair.second
        refreshProfile(goalState)
        return getSetpoint()
    }

    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using an exponential profile, a target position and a target velocity.
     */
    public fun getSetpoint(
        targetPosition: Angle,
        targetVelocity: AngularVelocity
    ): Setpoint<AngleDimension, VoltageDimension> {
        refreshProfile(State(targetPosition.siValue, targetVelocity.siValue))
        return getSetpoint()
    }

    private fun refreshProfile(goalState: State){
        val currentT = fpgaTimestamp()
        currentState = calculate(
            (currentT - previousT).inUnit(seconds),
            currentState,
            goalState
        )
        previousT = currentT
    }

    private fun getSetpoint() = Setpoint(
        Angle(currentState.position),
        velocityTargetFF.calculate(AngularVelocity(currentState.velocity))
    )

}