package frc.robot.hardware.subsystems.shooter.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.framework.ChargerRobot

@Suppress("unused")
class ShooterIOSim(
    private val topMotorSim: DCMotorSim,
    private val bottomMotorSim: DCMotorSim? = null,
): ShooterIO {
    private val motorSims = mutableListOf(topMotorSim).apply{
        if (bottomMotorSim != null){
            add(bottomMotorSim)
        }
    }

    private var _intakeVoltages: Array<Voltage> = motorSims.map{ Voltage(0.0) }.toTypedArray() // 1 voltage value per sim

    init{
        ChargerRobot.runPeriodically(addToFront = true) {
            motorSims.forEach{ motorSim ->
                motorSim.update(0.02)
            }
        }
    }

    override val hasGamepiece by ShooterLog.boolean{
        false
    }

    override val hasBeamBreakSensor by ShooterLog.boolean{
        false
    }

    override val intakeVoltages by ShooterLog.quantityList {
        _intakeVoltages.toList()
    }

    override val intakeSpeeds by ShooterLog.quantityList {
        motorSims.map{ it.angularVelocityRadPerSec.ofUnit(radians/seconds) }
    }

    override val intakeCurrents by ShooterLog.quantityList {
        motorSims.map{ it.currentDrawAmps.ofUnit(amps) }
    }

    override val intakeTemps by ShooterLog.doubleList {
        motorSims.map{ 0.0 }
    }

    override fun intake(voltage: Voltage) {
        _intakeVoltages[0] = voltage
        topMotorSim.setInputVoltage(voltage.inUnit(volts))
        if (bottomMotorSim != null){
            _intakeVoltages[1] = -voltage
            bottomMotorSim.setInputVoltage(voltage.inUnit(volts))
        }
    }
}