@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev.util

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel.PeriodicFrame
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController

internal fun addFollowers(
    revMotor: CANSparkBase,
    nonRevFollowerSetReference: MutableSet<SmartEncoderMotorController>,
    invert: Boolean,
    vararg followers: SmartEncoderMotorController
) {
    revMotor.apply{
        setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10)
        val revFollowers: MutableList<CANSparkBase> = mutableListOf()
        followers.forEach{ follower ->
            if (follower is CANSparkBase){
                revFollowers.add(follower)
                follower.follow(this, invert)

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
                if (invert){
                    follower.inverted = revMotor.inverted
                }else{
                    follower.inverted = !revMotor.inverted
                }
                nonRevFollowerSetReference.add(follower)
            }
        }
        revFollowers.forEach{
            it.burnFlash()
        }
    }
}