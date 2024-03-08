package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.times
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder





@Suppress("unused")
class PivotIOReal(
    private val motor: SmartEncoderMotorController,
    useOnboardPID: Boolean = true,
    private val encoderType: EncoderType,
    private val offset: Angle = Angle(0.0)
): PivotIO {

    init{
        ChargerRobot.runPeriodically{
            println(motor.encoder.angularPosition)
        }
    }

    sealed class EncoderType{
        /**
         * Represents an encoder type where the absolute encoder is "integrated"/plugged into the motor.
         * in other words, the motor is configured to return absolute encoder readings
         * from motor.encoder.angularPosition.
         */
        data object IntegratedAbsoluteEncoder: EncoderType()

        /**
         * Represents an encoder type where there is no absolute encoder,
         * and a relative encoder is used for positions.
         */
        data class IntegratedRelativeEncoder(
            val motorGearRatio: Double
        ): EncoderType()

        /**
         * Represents an encoder type where an external absolute encoder is used.
         */
        data class ExternalAbsoluteEncoder(
            val absoluteEncoder: PositionEncoder,
            val motorGearRatio: Double,
        ): EncoderType()
    }

    private val rioController: SuperPIDController<AngleDimension, VoltageDimension>? =
        if (useOnboardPID){
            null
        }else{
            getRioPIDController()
        }

    override val appliedVoltage by PivotLog.quantity{
        motor.appliedVoltage
    }
    
    override val angle by PivotLog.quantity{
        when (encoderType){
            EncoderType.IntegratedAbsoluteEncoder -> motor.encoder.angularPosition

            is EncoderType.ExternalAbsoluteEncoder -> encoderType.absoluteEncoder.angularPosition

            is EncoderType.IntegratedRelativeEncoder -> (motor.encoder.angularPosition / encoderType.motorGearRatio)
        } - offset
    }
    
    override val appliedCurrent by PivotLog.quantity{
        motor.appliedCurrent
    }
    
    override val tempCelsius by PivotLog.double{
        motor.tempCelsius
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.siValue)
    }
    
    override fun setAngleSetpoint(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        if (rioController != null){
            rioController.constants = pidConstants
            setVoltage(rioController.calculateOutput(position) + ffOutput)
        }else{
            when (encoderType){
                EncoderType.IntegratedAbsoluteEncoder -> {
                    motor.setAngularPosition(position, pidConstants)
                }

                is EncoderType.ExternalAbsoluteEncoder -> {
                    motor.setAngularPosition(
                        position,
                        pidConstants,
                        absoluteEncoder = encoderType.absoluteEncoder,
                        gearRatio = encoderType.motorGearRatio
                    )
                }

                is EncoderType.IntegratedRelativeEncoder -> {
                    motor.setAngularPosition(
                        // position is in the same field of reference as the position value,
                        // which is motor.encoder.angularPosition / gearRatio.
                        // thus, gear ratio is multiplied to restore it to the original position.
                        position * encoderType.motorGearRatio,
                        pidConstants
                    )
                }
            }
        }
    }
}