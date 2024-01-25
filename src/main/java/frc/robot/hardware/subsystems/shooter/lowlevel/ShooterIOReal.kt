package frc.robot.hardware.subsystems.shooter.lowlevel

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj.DigitalInput
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.setVoltage


@Suppress("unused")
class ShooterIOReal(
    private val beamBreakSensor: DigitalInput? = null,
    private val pivotOffset: Angle = Angle(0.0),

    private val leftMotor: SmartEncoderMotorController,
    private val rightMotor: SmartEncoderMotorController,
    private val pivotMotor: SmartEncoderMotorController,

    private val leftGearRatio: Double,
    private val rightGearRatio: Double,
    private val pivotGearRatio: Double,
): ShooterIO {
    override val hasGamepiece by ShooterLog.boolean{ beamBreakSensor?.get() ?: false }

    override val hasBeamBreakSensor by ShooterLog.boolean{ beamBreakSensor != null }


    // ShooterLog delegates handle calling updateInputs() and processInputs() automatically
    override val leftIntakeVoltage by ShooterLog.quantity{ leftMotor.appliedVoltage }
    override val rightIntakeVoltage by ShooterLog.quantity{ rightMotor.appliedVoltage }

    override val leftIntakeSpeed by ShooterLog.quantity{ leftMotor.encoder.angularVelocity / leftGearRatio }
    override val rightIntakeSpeed by ShooterLog.quantity{ rightMotor.encoder.angularVelocity / rightGearRatio }

    override val leftIntakeCurrent by ShooterLog.quantity{ leftMotor.appliedCurrent }
    override val rightIntakeCurrent by ShooterLog.quantity{ rightMotor.appliedCurrent }


    override val leftIntakeTemp by ShooterLog.double{ leftMotor.tempCelsius }
    override val rightIntakeTemp by ShooterLog.double{ rightMotor.tempCelsius }

    override fun spin(leftVoltage: Voltage, rightVoltage: Voltage) {
        // extension functions that accept voltages instead of doubles
        leftMotor.setVoltage(leftVoltage)
        rightMotor.setVoltage(rightVoltage)
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