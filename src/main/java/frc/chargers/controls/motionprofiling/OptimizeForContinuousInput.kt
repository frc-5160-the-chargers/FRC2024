package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.div
import frc.chargers.utils.math.inputModulus

/**
 * Modifies 2 motion profile states: the setpoint and the goal,
 * optimizing them to be appropriate for continuous input controllers.
 *
 * Note: You must call this whenever you use motion profiling with continuous input controllers;
 * otherwise, unexpected behavior might occur.
 */
fun <Pos: Dimension<*,*,*,*>, Vel: Dimension<*,*,*,*>> optimizeForContinuousInput(
    setpoint: MotionProfileState<Pos, Vel>,
    goal: MotionProfileState<Pos, Vel>,
    measurement: Quantity<Pos>,
    continuousInputRange: ClosedRange<Quantity<Pos>>
){
    val errorBound = (continuousInputRange.endInclusive - continuousInputRange.start) / 2.0
    val goalMinDistance =
        (goal.position - measurement)
            .inputModulus(-errorBound..errorBound)
    val setpointMinDistance =
        (setpoint.position - measurement)
            .inputModulus(-errorBound..errorBound)
    goal.position = goalMinDistance + measurement
    setpoint.position = setpointMinDistance + measurement
}