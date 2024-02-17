package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.Voltage
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController

@Suppress("unused")
class GroundIntakeIOReal(
    private val topMotor: SmartEncoderMotorController,
    private val bottomMotor: SmartEncoderMotorController? = null,
    private val conveyorMotor: SmartEncoderMotorController,

    private val conveyorVoltageMultiplier: Double = 1.0,

    private val intakeGearRatio: Double = 1.0,
    private val conveyorGearRatio: Double = 1.0,
): GroundIntakeIO {
    private val intakeMotors = if (bottomMotor != null){
        listOf(topMotor, bottomMotor)
    }else{
        listOf(topMotor)
    }

    override val intakeVoltages by GroundIntakeLog.quantityList{
        intakeMotors.map{ it.appliedVoltage }
    }

    override val intakeCurrents by GroundIntakeLog.quantityList{
        intakeMotors.map{ it.appliedCurrent }
    }

    override val intakeSpeeds by GroundIntakeLog.quantityList{
        intakeMotors.map{ it.encoder.angularVelocity / intakeGearRatio }
    }

    override val intakeTemps by GroundIntakeLog.doubleList{
        intakeMotors.map{ it.tempCelsius }
    }

    override val conveyorVoltage by GroundIntakeLog.quantity{
        conveyorMotor.appliedVoltage
    }

    override val conveyorCurrent by GroundIntakeLog.quantity{
        conveyorMotor.appliedCurrent
    }

    override val conveyorSpeed by GroundIntakeLog.quantity{
        conveyorMotor.encoder.angularVelocity / conveyorGearRatio
    }

    override fun intake(voltage: Voltage) {
        topMotor.setVoltage(voltage.siValue)
        bottomMotor?.setVoltage(-voltage.siValue)

        conveyorMotor.setVoltage(voltage.siValue * conveyorVoltageMultiplier)
    }
}