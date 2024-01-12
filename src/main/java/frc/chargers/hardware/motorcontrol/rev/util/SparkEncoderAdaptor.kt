@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.minutes
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.*
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.revertIfInvalid


public class SparkEncoderAdaptor(
    private val revEncoder: MotorFeedbackSensor
) : ResettableEncoder {
    private var previousPosition = getPosition()
    private var previousVelocity = AngularVelocity(0.0)

    init{
        require (
            revEncoder is AbsoluteEncoder || revEncoder is RelativeEncoder
        ){"Encoder type of spark max is invalid: internal error."}
    }


    override fun setZero(newZero: Angle){
        when (revEncoder){
            is AbsoluteEncoder -> revEncoder.setZeroOffset(newZero.inUnit(rotations))
            is RelativeEncoder -> revEncoder.setPosition(newZero.inUnit(rotations))
        }
    }

    override val angularPosition: Angle
        get() = getPosition()
            .revertIfInvalid(previousPosition)
            .also{ previousPosition = it }

    override val angularVelocity: AngularVelocity
        get() = getVelocity()
            .revertIfInvalid(previousVelocity)
            .also{ previousVelocity = it }




    private fun getPosition(): Angle = when (revEncoder){
        is AbsoluteEncoder -> revEncoder.position.ofUnit(rotations)
        is RelativeEncoder -> revEncoder.position.ofUnit(rotations)
        else -> error("Invalid encoder type for spark max")
    }

    private fun getVelocity(): AngularVelocity = when (revEncoder){
        is AbsoluteEncoder -> revEncoder.velocity.ofUnit(rotations / seconds)
        is RelativeEncoder -> revEncoder.velocity.ofUnit(rotations / minutes)
        else -> error("Invalid encoder type for spark max")
    }
}