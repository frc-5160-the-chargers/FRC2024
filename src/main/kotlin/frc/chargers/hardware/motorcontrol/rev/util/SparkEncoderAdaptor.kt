@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.minutes
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.*
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.filterInvalid


public class SparkEncoderAdaptor(private val motor: CANSparkBase) : ResettableEncoder {
    private val relativeEncoder = motor.encoder
    private val absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
    private var useAbsoluteEncoder = false

    init {
        motor.pidController.setFeedbackDevice(relativeEncoder)
    }

    // here, null represents a setting that should not be changed
    // from the original.
    fun configure(
        useAbsoluteEncoder: Boolean? = null,
        averageDepth: Int? = null,
        inverted: Boolean? = null
    ){
        if (useAbsoluteEncoder != null){
            this.useAbsoluteEncoder = useAbsoluteEncoder
            if (this.useAbsoluteEncoder){
                motor.pidController.setFeedbackDevice(absoluteEncoder)
            }else{
                motor.pidController.setFeedbackDevice(relativeEncoder)
            }
        }

        if (averageDepth != null){
            if (this.useAbsoluteEncoder){
                absoluteEncoder.averageDepth = averageDepth
            }else{
                relativeEncoder.averageDepth = averageDepth
            }
        }

        if (inverted != null){
            if (this.useAbsoluteEncoder){
                absoluteEncoder.inverted = inverted
            }else{
                relativeEncoder.inverted = inverted
            }
        }
    }

    override fun setZero(newZero: Angle){
        if (useAbsoluteEncoder){
            absoluteEncoder.setZeroOffset(newZero.inUnit(rotations))
        }else{
            relativeEncoder.setPosition(newZero.inUnit(rotations))
        }
    }

    override val angularPosition: Angle by filterInvalid {
        if (useAbsoluteEncoder){
            absoluteEncoder.position.ofUnit(rotations)
        }else{
            relativeEncoder.position.ofUnit(rotations)
        }
    }

    override val angularVelocity: AngularVelocity by filterInvalid {
        if (useAbsoluteEncoder){
            absoluteEncoder.velocity.ofUnit(rotations / minutes)
        }else{
            relativeEncoder.velocity.ofUnit(rotations / seconds)
        }
    }
}