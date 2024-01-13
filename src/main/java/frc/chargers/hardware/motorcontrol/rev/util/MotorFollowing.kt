@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev.util

import com.revrobotics.CANSparkLowLevel.*
import com.revrobotics.CANSparkBase
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.rev.DISABLED_PERIODIC_FRAME_STRATEGY
import frc.chargers.hardware.motorcontrol.rev.SLOW_PERIODIC_FRAME_STRATEGY

internal fun addFollowers(
    revMotor: CANSparkBase,
    nonRevFollowerSet: MutableSet<SmartEncoderMotorController>,
    vararg followers: SmartEncoderMotorController
) {
    revMotor.apply{
        setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10)
        val revFollowers: MutableList<CANSparkBase> = mutableListOf()
        followers.forEach{ follower ->
            if (follower is CANSparkBase){
                revFollowers.add(follower)
                follower.follow(
                    this,
                    // determines whether to invert the follower;
                    // cast necessary to avoid overload resolution ambiguity
                    (follower as CANSparkBase).inverted != revMotor.inverted
                )

                // configures frame periods of each follower, to reduce device latency
                // doesn't need a safe config; as only cost is a little bit more bus utilization
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, SLOW_PERIODIC_FRAME_STRATEGY)
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, SLOW_PERIODIC_FRAME_STRATEGY)
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, SLOW_PERIODIC_FRAME_STRATEGY)
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, DISABLED_PERIODIC_FRAME_STRATEGY)
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, DISABLED_PERIODIC_FRAME_STRATEGY)
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus5, DISABLED_PERIODIC_FRAME_STRATEGY)
                follower.setPeriodicFramePeriod(PeriodicFrame.kStatus6, DISABLED_PERIODIC_FRAME_STRATEGY)
            }else{
                nonRevFollowerSet.add(follower)
            }
        }
        revFollowers.forEach{
            it.burnFlash()
        }
    }
}