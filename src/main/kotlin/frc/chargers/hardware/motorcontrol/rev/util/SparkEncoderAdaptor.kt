@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.minutes
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.*
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.utils.filterInvalid


public class SparkEncoderAdaptor(
    private val revEncoder: MotorFeedbackSensor
) : ResettableEncoder {

    public constructor(motor: CANSparkBase, encoderType: SparkEncoderType): this(
        when (encoderType){
            is SparkEncoderType.Regular -> motor.encoder.apply{
                motor.pidController.setFeedbackDevice(this@apply)
                // property access syntax setters
                if (encoderType.averageDepth != null){
                    averageDepth = encoderType.averageDepth
                }
                if (encoderType.inverted != null){
                    inverted = encoderType.inverted
                }
            }

            is SparkEncoderType.Quadrature -> when (motor){
                is CANSparkFlex -> motor.getExternalEncoder(encoderType.countsPerRev)

                is CANSparkMax -> motor.getAlternateEncoder(encoderType.countsPerRev)

                else -> error("Invalid motor type.")
            }.apply{
                motor.pidController.setFeedbackDevice(this@apply)
                // property access syntax setters
                if (encoderType.encoderMeasurementPeriod != null){
                    measurementPeriod = encoderType.encoderMeasurementPeriod.inUnit(milli.seconds).toInt()
                }
                if (encoderType.averageDepth != null){
                    averageDepth = encoderType.averageDepth
                }
                if (encoderType.inverted != null){
                    inverted = encoderType.inverted
                }
            }

            is SparkEncoderType.DutyCycle -> motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).apply{
                motor.pidController.setFeedbackDevice(this@apply)
                // property access syntax setters(replace setAverageDepth and getAverageDepth)
                if (encoderType.averageDepth != null){
                    averageDepth = encoderType.averageDepth
                }
                if (encoderType.inverted != null){
                    inverted = encoderType.inverted
                }
            }
        }
    )


    override fun setZero(newZero: Angle){
        when (revEncoder){
            is AbsoluteEncoder -> revEncoder.setZeroOffset(newZero.inUnit(rotations))
            is RelativeEncoder -> revEncoder.setPosition(newZero.inUnit(rotations))
        }
    }

    override val angularPosition: Angle by filterInvalid{
        when (revEncoder){
            is AbsoluteEncoder -> revEncoder.position.ofUnit(rotations)
            is RelativeEncoder -> revEncoder.position.ofUnit(rotations)
            else -> error("Invalid encoder type for spark max")
        }
    }

    override val angularVelocity: AngularVelocity by filterInvalid{
        when (revEncoder){
            is AbsoluteEncoder -> revEncoder.velocity.ofUnit(rotations / seconds)
            is RelativeEncoder -> revEncoder.velocity.ofUnit(rotations / minutes)
            else -> error("Invalid encoder type for spark max")
        }
    }
}