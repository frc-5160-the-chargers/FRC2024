@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.motionprofiling

import edu.wpi.first.math.trajectory.ExponentialProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.utils.math.inputModulus

/**
 * Optimizes the setpoint and goal of a trapezoidal motion profile
 * for continuous input.
 */
internal fun optimizeMotionProfileTargets(
    setpoint: TrapezoidProfile.State,
    goal: TrapezoidProfile.State,
    continuousInputRange: ClosedRange<Double>,
    mechanismMeasurement: Double
){
    val errorBound = (continuousInputRange.endInclusive - continuousInputRange.start) / 2.0
    val goalMinDistance =
        (goal.position - mechanismMeasurement)
            .inputModulus(-errorBound..errorBound)
    val setpointMinDistance =
        (setpoint.position - mechanismMeasurement)
            .inputModulus(-errorBound..errorBound)
    setpoint.position =  goalMinDistance + mechanismMeasurement
    goal.position =  setpointMinDistance + mechanismMeasurement
}

/**
 * Optimizes the setpoint and goal of a exponential motion profile
 * for continuous input.
 */
internal fun optimizeMotionProfileTargets(
    setpoint: ExponentialProfile.State,
    goal: ExponentialProfile.State,
    continuousInputRange: ClosedRange<Double>,
    mechanismMeasurement: Double
): Pair<ExponentialProfile.State, ExponentialProfile.State>{
    val errorBound = (continuousInputRange.endInclusive - continuousInputRange.start) / 2.0
    val goalMinDistance =
        (goal.position - mechanismMeasurement)
            .inputModulus(-errorBound..errorBound)
    val setpointMinDistance =
        (setpoint.position - mechanismMeasurement)
            .inputModulus(-errorBound..errorBound)
    val newSetpoint = ExponentialProfile.State(goalMinDistance + mechanismMeasurement, goal.velocity)
    val newGoal = ExponentialProfile.State(setpointMinDistance + mechanismMeasurement, setpoint.velocity)

    return Pair(newSetpoint, newGoal)
}