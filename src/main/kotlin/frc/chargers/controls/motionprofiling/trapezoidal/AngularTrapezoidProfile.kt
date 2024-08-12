@file:Suppress("unused")
package frc.chargers.controls.motionprofiling.trapezoidal

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.motionprofiling.MotionProfileState

class AngularTrapezoidProfile(
    val maxVelocity: AngularVelocity,
    val maxAcceleration: AngularAcceleration
): AngularMotionProfile {
    private val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity.siValue, maxAcceleration.siValue))

    override fun calculate(
        setpoint: MotionProfileState<AngleDimension, AngularVelocityDimension>,
        goal: MotionProfileState<AngleDimension, AngularVelocityDimension>,
        dt: Time,
    ): AngularMotionProfileState {

        val profileState = profile.calculate(
            dt.inUnit(seconds),
            setpoint.toTrapezoidalState(),
            goal.toTrapezoidalState()
        )

        return AngularMotionProfileState(
            Angle(profileState.position),
            AngularVelocity(profileState.velocity)
        )
    }

}