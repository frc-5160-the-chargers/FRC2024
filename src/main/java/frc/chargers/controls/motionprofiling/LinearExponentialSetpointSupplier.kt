@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import frc.chargers.controls.Setpoint
import frc.chargers.controls.SetpointSupplier
import frc.chargers.wpilibextensions.fpgaTimestamp

/**
 * A [SetpointSupplier] instance which derives Linear [Setpoint]s
 * (consisting of the setpoint(Distance) and a feedforward value(Voltage))
 * from an [ExponentialProfile].
 */
public class LinearExponentialSetpointSupplier(
    profileConstraints: Constraints,
    /**
     * The feedforward helps the mechanism reach the velocity output of the motion profile.
     */
    private val velocityFFEquation: (Velocity) -> Voltage = { Voltage(0.0) },
    startingPosition: Distance = Distance(0.0),
    startingVelocity: Velocity = Velocity(0.0)
): ExponentialProfile(profileConstraints), SetpointSupplier<DistanceDimension, VoltageDimension>{

    private var currentState = State(startingPosition.siValue, startingVelocity.siValue)
    private var previousT = fpgaTimestamp()


    /**
     * Fetches the current position that the [SetpointSupplier]
     * wants to direct the mechanism to.
     */
    public val setpointPosition: Distance
        get() = Distance(currentState.position)

    /**
     * Fetches the current velocity that the [SetpointSupplier]
     * wants to direct the mechanism to.
     */
    public val setpointVelocity: Velocity
        get() = Velocity(currentState.velocity)


    override fun calculateSetpoint(target: Distance): Setpoint<DistanceDimension, VoltageDimension> =
        getSetpoint(target, Velocity(0.0))

    override fun calculateSetpoint(
        target: Distance,
        continuousInputRange: ClosedRange<Distance>,
        measurement: Distance
    ): Setpoint<DistanceDimension, VoltageDimension> =
        calculateSetpoint(target, Velocity(0.0), continuousInputRange, measurement)


    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity,
     * with a specified continuous input range.
     */
    public fun calculateSetpoint(
        targetPosition: Distance,
        targetVelocity: Velocity,
        continuousInputRange: ClosedRange<Distance>,
        measurement: Distance
    ): Setpoint<DistanceDimension, VoltageDimension>{
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
        targetPosition: Distance,
        targetVelocity: Velocity
    ): Setpoint<DistanceDimension, VoltageDimension> {
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
        Distance(currentState.position),
        velocityFFEquation(Velocity(currentState.velocity))
    )

}