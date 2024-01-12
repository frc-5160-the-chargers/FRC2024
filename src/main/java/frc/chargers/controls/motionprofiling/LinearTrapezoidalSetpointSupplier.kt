@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.Setpoint
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints


/**
 * A [SetpointSupplier] instance which derives Linear [Setpoint]s
 * (consisting of the setpoint([Distance]) and a feedforward value([Voltage]))
 * from a [TrapezoidProfile].
 */
public class LinearTrapezoidalSetpointSupplier(
    profileConstraints: LinearMotionConstraints,
    /**
     * The feedforward helps the mechanism reach the velocity output of the motion profile.
     */
    private val velocityTargetFF: Feedforward<VelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) },
    startingState: State = State()
): TrapezoidProfile(profileConstraints.siValue), SetpointSupplier<DistanceDimension, VoltageDimension> {

    public constructor(
        maxVelocity: Velocity,
        maxAcceleration: Acceleration,
        feedforward: Feedforward<VelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
    ): this(LinearMotionConstraints(maxVelocity,maxAcceleration), feedforward)

    public constructor(
        constraints: Constraints,
        feedforward: Feedforward<VelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) }
    ): this(LinearMotionConstraints(constraints), feedforward)



    private var currentState = startingState
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


    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity of 0.
     */
    override fun calculateSetpoint(target: Distance): Setpoint<DistanceDimension, VoltageDimension> =
        calculateSetpoint(target, Velocity(0.0))

    /**
     * Calculates the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position, and a velocity of 0,
     * with continuous input support.
     */
    override fun calculateSetpoint(
        target: Distance,
        continuousInputRange: ClosedRange<Distance>,
        measurement: Distance
    ): Setpoint<DistanceDimension, VoltageDimension> =
        calculateSetpoint(
            target, continuousInputRange = continuousInputRange, measurement = measurement
        )


    /**
     * Calculates the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity.
     */
    public fun calculateSetpoint(
        targetPosition: Distance,
        targetVelocity: Velocity
    ): Setpoint<DistanceDimension, VoltageDimension> {
        refreshProfile(State(targetPosition.siValue,targetVelocity.siValue))
        return getSetpoint()
    }

    /**
     * Calculates the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity,
     * with continuous input support.
     */
    public fun calculateSetpoint(
        targetPosition: Distance,
        targetVelocity: Velocity,
        continuousInputRange: ClosedRange<Distance>,
        mechanismMeasurement: Distance
    ): Setpoint<DistanceDimension, VoltageDimension>{
        val goalState = State(targetPosition.siValue, targetVelocity.siValue)
        optimizeMotionProfileTargets(
            currentState, goalState,
            continuousInputRange.start.siValue..continuousInputRange.endInclusive.siValue,
            mechanismMeasurement.siValue
        )
        refreshProfile(goalState)
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
        velocityTargetFF.calculate(Velocity(currentState.velocity))
    )

}