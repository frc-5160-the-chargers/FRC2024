package frc.chargers.controls.motionprofiling

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Time
import edu.wpi.first.math.trajectory.ExponentialProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile

typealias AngularMotionProfile = MotionProfile<AngleDimension, AngularVelocityDimension>

typealias AngularMotionProfileState = MotionProfileState<AngleDimension, AngularVelocityDimension>


typealias LinearMotionProfile = MotionProfile<DistanceDimension, VelocityDimension>

typealias LinearMotionProfileState = MotionProfileState<DistanceDimension, VelocityDimension>


/**
 * Represents a Generic motion profile, that can calculate an intermediate state
 * between a goal and a setpoint, acheiving smoother and more optimal motion.
 *
 * This is most commonly paired with a controller of some kind; like PID or LQR.
 */
fun interface MotionProfile <Pos: Dimension<*,*,*,*>, Vel: Dimension<*,*,*,*>>{
    fun calculate(dt: Time, setpoint: MotionProfileState<Pos,Vel>, goal: MotionProfileState<Pos,Vel>): MotionProfileState<Pos,Vel>
}


data class MotionProfileState <Pos: Dimension<*,*,*,*>, Vel: Dimension<*,*,*,*>>(
    var position: Quantity<Pos> = Quantity(0.0),
    var velocity: Quantity<Vel> = Quantity(0.0)
){
    fun toTrapezoidalState(): TrapezoidProfile.State =
        TrapezoidProfile.State(position.siValue, velocity.siValue)

    fun toExponentialState(): ExponentialProfile.State =
        ExponentialProfile.State(position.siValue, velocity.siValue)
}

