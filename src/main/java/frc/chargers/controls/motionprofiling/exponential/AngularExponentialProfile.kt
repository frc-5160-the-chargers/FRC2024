@file:Suppress("unused", "MemberVisibilityCanBePrivate", "CanBeParameter")
package frc.chargers.controls.motionprofiling.exponential

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.utils.math.units.VoltagePerAngularAcceleration
import frc.chargers.utils.math.units.VoltagePerAngularVelocity


class AngularExponentialProfile(
    val constraints: ExponentialProfile.Constraints
): AngularMotionProfile {

    companion object{
        /**
         * @see ExponentialProfile.Constraints.fromStateSpace
         */
        fun fromStateSpace(
            maxInput: Voltage,
            a: Double,
            b: Double
        ) = AngularExponentialProfile(
            ExponentialProfile.Constraints.fromStateSpace(maxInput.siValue, a, b)
        )

        /**
         * @see ExponentialProfile.Constraints.fromCharacteristics
         */
        fun fromCharacteristics(
            maxInput: Voltage,
            kV: Double,
            kA: Double
        ) = AngularExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(maxInput.siValue, kV, kA)
        )

        /**
         * @see ExponentialProfile.Constraints.fromCharacteristics
         */
        fun fromCharacteristics(
            maxInput: Voltage,
            kV: VoltagePerAngularVelocity,
            kA: VoltagePerAngularAcceleration
        ) = AngularExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(maxInput.siValue, kV.siValue, kA.siValue)
        )
    }

    private val profile = ExponentialProfile(constraints)

    override fun calculate(
        dt: Time,
        setpoint: AngularMotionProfileState,
        goal: AngularMotionProfileState,
    ): AngularMotionProfileState {

        val profileState = profile.calculate(
            dt.inUnit(seconds),
            setpoint.toExponentialState(),
            goal.toExponentialState()
        )

        return AngularMotionProfileState(
            Angle(profileState.position),
            AngularVelocity(profileState.velocity)
        )
    }

}