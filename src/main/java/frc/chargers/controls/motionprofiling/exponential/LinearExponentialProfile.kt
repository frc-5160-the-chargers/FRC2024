@file:Suppress("unused", "CanBeParameter", "MemberVisibilityCanBePrivate")
package frc.chargers.controls.motionprofiling.exponential

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.ExponentialProfile
import frc.chargers.controls.motionprofiling.LinearMotionProfile
import frc.chargers.controls.motionprofiling.LinearMotionProfileState
import frc.chargers.utils.math.units.VoltagePerAcceleration
import frc.chargers.utils.math.units.VoltagePerVelocity


class LinearExponentialProfile(
    val constraints: ExponentialProfile.Constraints
): LinearMotionProfile {
    companion object{
        /**
         * @see ExponentialProfile.Constraints.fromStateSpace
         */
        fun fromStateSpace(
            maxInput: Voltage,
            a: Double,
            b: Double
        ) = LinearExponentialProfile(
            ExponentialProfile.Constraints.fromStateSpace(maxInput.siValue, a, b)
        )

        /**
         * @see ExponentialProfile.Constraints.fromCharacteristics
         */
        fun fromCharacteristics(
            maxInput: Voltage,
            kV: Double,
            kA: Double
        ) = LinearExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(maxInput.siValue, kV, kA)
        )

        /**
         * @see ExponentialProfile.Constraints.fromCharacteristics
         */
        fun fromCharacteristics(
            maxInput: Voltage,
            kV: VoltagePerVelocity,
            kA: VoltagePerAcceleration
        ) = LinearExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(maxInput.siValue, kV.siValue, kA.siValue)
        )
    }

    private val profile = ExponentialProfile(constraints)

    override fun calculate(
        dt: Time,
        setpoint: LinearMotionProfileState,
        goal: LinearMotionProfileState,
    ): LinearMotionProfileState {

        val profileState = profile.calculate(
            dt.inUnit(seconds),
            setpoint.toExponentialState(),
            goal.toExponentialState()
        )

        return LinearMotionProfileState(
            Distance(profileState.position),
            Velocity(profileState.velocity)
        )
    }

}