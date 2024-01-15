package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj.DigitalInput
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.motorcontrol.setVoltage


@Suppress("unused")
class ShooterIOReal(
    private val beamBreakSensor: DigitalInput? = null,
    private val pivotOffset: Angle = Angle(0.0),

    private val leftMotors: SmartEncoderMotorController,
    private val rightMotors: SmartEncoderMotorController,
    private val pivotMotors: SmartEncoderMotorController,

    private val leftGearRatio: Double,
    private val rightGearRatio: Double,
    private val pivotGearRatio: Double,
): ShooterIO {



    override val hasGamepiece by ShooterLog.boolean{ beamBreakSensor?.get() ?: false }

    override val hasBeamBreakSensor by ShooterLog.boolean{ beamBreakSensor != null }


    // ShooterLog delegates handle calling updateInputs() and processInputs() automatically
    override val leftIntakeVoltage by ShooterLog.quantity{ leftMotors.appliedVoltage }
    override val rightIntakeVoltage by ShooterLog.quantity{ rightMotors.appliedVoltage }

    override val leftIntakeSpeed by ShooterLog.quantity{ leftMotors.encoder.angularVelocity / leftGearRatio }
    override val rightIntakeSpeed by ShooterLog.quantity{ rightMotors.encoder.angularVelocity / rightGearRatio }

    override val leftIntakeCurrent by ShooterLog.quantity{ leftMotors.appliedCurrent }
    override val rightIntakeCurrent by ShooterLog.quantity{ rightMotors.appliedCurrent }


    override val leftIntakeTemp by ShooterLog.double{ leftMotors.tempCelsius }
    override val rightIntakeTemp by ShooterLog.double{ rightMotors.tempCelsius }

    override fun spin(leftVoltage: Voltage, rightVoltage: Voltage) {
        // extension functions that accept voltages instead of doubles
        leftMotors.setVoltage(leftVoltage)
        rightMotors.setVoltage(rightVoltage)
    }

    override val pivotVoltage by ShooterLog.quantity{ pivotMotors.appliedVoltage }
    override val pivotPosition by ShooterLog.quantity{ (pivotMotors.encoder.angularPosition / pivotGearRatio) - pivotOffset }
    override val pivotCurrent by ShooterLog.quantity{ pivotMotors.appliedCurrent }
    override val pivotTemp by ShooterLog.double{ pivotMotors.tempCelsius }

    override fun setPivotVoltage(voltage: Voltage) {
        pivotMotors.setVoltage(voltage)
    }

    override fun setPivotPosition(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        pivotMotors.setAngularPosition(position, pidConstants, extraVoltage = ffOutput)
    }
}