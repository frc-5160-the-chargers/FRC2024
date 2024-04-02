package frc.robot.hardware.subsystems.shooter.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DigitalInput
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController

/**
 * An implementation of the low level component of the shooter, on the real robot.
 */
@Suppress("unused")
class ShooterIOReal(
    private val beamBreakSensor: DigitalInput? = null,
    private val topMotor: SmartEncoderMotorController,
    private val bottomMotor: SmartEncoderMotorController? = null,
    private val gearRatio: Double = 1.0,
): ShooterIO {
    private val allMotors = mutableListOf(topMotor).apply{
        if (bottomMotor != null){
            add(bottomMotor)
        }
    }

    override val hasNote by ShooterLog.boolean{
        if (beamBreakSensor == null){
            false
        }else{
            !beamBreakSensor.get()
        }
    }

    override val hasNoteDetector by ShooterLog.boolean{
        beamBreakSensor != null
    }

    override val intakeVoltages by ShooterLog.quantityList{
        allMotors.map{ it.appliedVoltage * it.get() }
    }

    override val intakeSpeeds by ShooterLog.quantityList{
        allMotors.map{ it.encoder.angularVelocity / gearRatio }
    }

    override val intakeCurrents by ShooterLog.quantityList{
        allMotors.map{ it.appliedCurrent }
    }

    override val intakeTemps by ShooterLog.doubleList{
        allMotors.map{ it.tempCelsius }
    }

    override fun setIntakeVoltage(voltage: Voltage) {
        topMotor.setVoltage(voltage.inUnit(volts))
        bottomMotor?.setVoltage(-voltage.inUnit(volts))
    }

    override fun setVelocity(velocity: AngularVelocity, pidConstants: PIDConstants, feedforwardVoltage: Voltage) {
        topMotor.setAngularVelocity(velocity * gearRatio, pidConstants, feedforwardVoltage)
        bottomMotor?.setAngularVelocity(velocity * gearRatio, pidConstants, feedforwardVoltage)
    }
}