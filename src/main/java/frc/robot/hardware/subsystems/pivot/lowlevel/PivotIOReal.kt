package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.commands.runOnceCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.robot.hardware.inputdevices.OperatorInterface
import frc.robot.hardware.subsystems.pivot.PivotEncoderType


@Suppress("unused")
class PivotIOReal(
    private val motor: SmartEncoderMotorController,
    useOnboardPID: Boolean = true,
    private val encoderType: PivotEncoderType,
): PivotIO {
    private var offset = Angle(0.0)

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
            is PivotEncoderType.IntegratedAbsoluteEncoder -> motor.encoder.angularPosition

            is PivotEncoderType.ExternalAbsoluteEncoder -> encoderType.absoluteEncoder.angularPosition

            is PivotEncoderType.IntegratedRelativeEncoder -> (motor.encoder.angularPosition / encoderType.motorGearRatio)
        } - offset
    }
    
    override val appliedCurrent by PivotLog.quantity{
        motor.appliedCurrent
    }
    
    override val tempCelsius by PivotLog.double{
        motor.tempCelsius
    }




    init{
        when(encoderType){
            is PivotEncoderType.IntegratedRelativeEncoder -> {
                offset = this.angle - encoderType.startingAngle

                // allows debug heading zeroing here
                OperatorInterface.resetPivotAngleTrigger.onTrue(
                    runOnceCommand{ offset = this.angle + offset - encoderType.startingAngle }
                )
            }

            is PivotEncoderType.ExternalAbsoluteEncoder -> {
                this.offset = encoderType.offset
            }

            is PivotEncoderType.IntegratedAbsoluteEncoder -> {
                this.offset = encoderType.offset
            }
        }
    }




    /*
    override fun setBrakeMode(shouldBrake: Boolean) {
        if (motor is CANSparkBase){
            if (shouldBrake){
                motor.idleMode = CANSparkBase.IdleMode.kBrake
            }else{
                motor.idleMode = CANSparkBase.IdleMode.kCoast
            }
        }else if (motor is TalonFX){
            val configuration = TalonFXConfiguration()
            motor.configurator.refresh(configuration)
            if (shouldBrake){
                configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake
            }else{
                configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast
            }
            motor.configurator.apply(configuration)
        }
    }
     */

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.siValue)
    }
    
    override fun setAngleSetpoint(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        if (rioController != null){
            rioController.constants = pidConstants
            var voltageOut = rioController.calculateOutput(position) + ffOutput
            if (DriverStation.isAutonomous()){
                voltageOut = voltageOut.coerceAtMost(4.volts)
            }
            setVoltage(voltageOut)
        }else{
            when (encoderType){
                is PivotEncoderType.IntegratedAbsoluteEncoder -> motor.setAngularPosition(position, pidConstants)

                is PivotEncoderType.ExternalAbsoluteEncoder -> motor.setAngularPosition(
                    position,
                    pidConstants,
                    absoluteEncoder = encoderType.absoluteEncoder,
                    gearRatio = encoderType.motorGearRatio
                )

                is PivotEncoderType.IntegratedRelativeEncoder -> motor.setAngularPosition(
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