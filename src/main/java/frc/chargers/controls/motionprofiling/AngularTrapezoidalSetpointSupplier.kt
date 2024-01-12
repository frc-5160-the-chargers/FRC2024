@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.Setpoint
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.feedforward.Feedforward
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.motion.AngularMotionConstraints

/**
 * A [SetpointSupplier] instance which derives Angular [Setpoint]s
 * (consisting of the setpoint(Angle) and a feedforward value(Voltage))
 * from a [TrapezoidProfile].
 */
public class AngularTrapezoidalSetpointSupplier(
    profileConstraints: AngularMotionConstraints,
    /**
     * The feedforward helps the mechanism reach the velocity output of the motion profile.
     */
    private val velocityTargetFF: Feedforward<AngularVelocityDimension, VoltageDimension> = Feedforward{ Voltage(0.0) },
    startingState: State = State()
): TrapezoidProfile(profileConstraints.siValue), SetpointSupplier<AngleDimension, VoltageDimension> {

    public constructor(
        maxVelocity: AngularVelocity,
        maxAcceleration: AngularAcceleration,
        feedforward: Feedforward<AngularVelocityDimension, VoltageDimension> = Feedforward { Voltage(0.0) }
    ) : this(AngularMotionConstraints(maxVelocity, maxAcceleration), feedforward)

    public constructor(
        constraints: Constraints,
        feedforward: Feedforward<AngularVelocityDimension, VoltageDimension> = Feedforward { Voltage(0.0) }
    ) : this(AngularMotionConstraints(constraints), feedforward)


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
     * using a trapezoid profile, a target position and a target velocity of 0.
     */
    override fun calculateSetpoint(target: Angle): Setpoint<AngleDimension, VoltageDimension>  =
        calculateSetpoint(target, AngularVelocity(0.0))

    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity of 0.
     */
    override fun calculateSetpoint(
        target: Angle,
        continuousInputRange: ClosedRange<Angle>,
        measurement: Angle
    ): Setpoint<AngleDimension, VoltageDimension> =
        calculateSetpoint(target, continuousInputRange = continuousInputRange, measurement = measurement)



    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity.
     */
    public fun calculateSetpoint(
        targetPosition: Angle,
        targetVelocity: AngularVelocity,
    ): Setpoint<AngleDimension, VoltageDimension> {
        refreshProfile(State(targetPosition.siValue, targetVelocity.siValue))
        return getSetpoint()
    }


    /**
     * Fetches the appropriate [Setpoint], containing target position and FF output,
     * using a trapezoid profile, a target position and a target velocity,
     * with a specified continuous input range.
     */
    public fun calculateSetpoint(
        targetPosition: Angle,
        targetVelocity: AngularVelocity,
        continuousInputRange: ClosedRange<Angle>,
        mechanismMeasurement: Angle
    ): Setpoint<AngleDimension, VoltageDimension>{
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
        Angle(currentState.position),
        velocityTargetFF.calculate(AngularVelocity(currentState.velocity))
    )

}