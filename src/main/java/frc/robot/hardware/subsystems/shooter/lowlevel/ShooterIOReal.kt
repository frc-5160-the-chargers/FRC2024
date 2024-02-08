package frc.robot.hardware.subsystems.shooter.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DigitalInput
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.setVoltage


@Suppress("unused")
class ShooterIOReal(
    private val beamBreakSensor: DigitalInput? = null,
    private val pivotOffset: Angle = Angle(0.0),

    private val topMotor: SmartEncoderMotorController,
    private val pivotMotor: SmartEncoderMotorController,

    private val intakeGearRatio: Double,
    private val pivotGearRatio: Double,
): ShooterIO {
    override val hasGamepiece by ShooterLog.boolean{ beamBreakSensor?.get() ?: false }
    override val hasBeamBreakSensor by ShooterLog.boolean{ beamBreakSensor != null }

    override val intakeVoltages by ShooterLog.quantityList{
        listOf(topMotor.appliedVoltage)
    }

    override val intakeSpeeds by ShooterLog.quantityList{
        listOf(topMotor.encoder.angularVelocity / intakeGearRatio)
    }

    override val intakeCurrents by ShooterLog.quantityList{
        listOf(topMotor.appliedCurrent)
    }

    override val intakeTemps by ShooterLog.doubleList{
        listOf(topMotor.tempCelsius)
    }

    override fun setVoltage(voltage: Voltage) {
        topMotor.setVoltage(voltage.inUnit(volts))
    }



    override val pivotVoltage by ShooterLog.quantity{ pivotMotor.appliedVoltage }
    override val pivotPosition by ShooterLog.quantity{ (pivotMotor.encoder.angularPosition / pivotGearRatio) - pivotOffset }
    override val pivotCurrent by ShooterLog.quantity{ pivotMotor.appliedCurrent }
    override val pivotTemp by ShooterLog.double{ pivotMotor.tempCelsius }

    override fun setPivotVoltage(voltage: Voltage) {
        pivotMotor.setVoltage(voltage)
    }

    override fun setPivotPosition(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        pivotMotor.setAngularPosition(position, pidConstants, extraVoltage = ffOutput)
    }
}