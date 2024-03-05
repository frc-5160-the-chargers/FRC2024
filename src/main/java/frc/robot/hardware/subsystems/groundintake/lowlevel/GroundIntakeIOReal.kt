package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController

@Suppress("unused")
class GroundIntakeIOReal(
    private val topMotor: SmartEncoderMotorController,
    private val bottomMotor: SmartEncoderMotorController? = null,
    private val conveyorMotor: SmartEncoderMotorController? = null,

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
        conveyorMotor?.appliedVoltage ?: 0.volts
    }

    override val conveyorCurrent by GroundIntakeLog.quantity{
        conveyorMotor?.appliedCurrent ?: 0.amps
    }

    override val conveyorSpeed by GroundIntakeLog.quantity{
        if (conveyorMotor != null){
            conveyorMotor.encoder.angularVelocity / conveyorGearRatio
        }else{
            0.radians / 0.seconds
        }
    }

    override fun setIntakeVoltage(voltage: Voltage) {
        topMotor.setVoltage(voltage.siValue)
        bottomMotor?.setVoltage(-voltage.siValue)
    }

    override fun setConveyorVoltage(voltage: Voltage) {
        conveyorMotor?.setVoltage(voltage.siValue)
    }
}