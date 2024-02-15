package frc.robot.hardware.subsystems.groundintake.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.framework.ChargerRobot

@Suppress("unused")
class GroundIntakeIOSim(
    private val topMotorSim: DCMotorSim,
    private val bottomMotorSim: DCMotorSim? = null,
    private val conveyorMotorSim: DCMotorSim,
    private val conveyorVoltageMultiplier: Double = 1.0
): GroundIntakeIO {
    private val intakeSims: List<DCMotorSim> = mutableListOf(topMotorSim).apply{
        if (bottomMotorSim != null){
            add(bottomMotorSim)
        }
    }

    // must be array to allow mutability but no adding values
    private val _intakeVoltages = intakeSims.map{ 0.volts }.toTypedArray()
    private var _conveyorVoltage = 0.volts

    init{
        ChargerRobot.runPeriodically(addToFront = true) {
            intakeSims.forEach { motorSim ->
                motorSim.update(0.02)
            }
            conveyorMotorSim.update(0.02)
        }
    }

    override val intakeVoltages by GroundIntakeLog.quantityList{
        _intakeVoltages.toList()
    }

    override val intakeSpeeds by GroundIntakeLog.quantityList{
        intakeSims.map{ it.angularVelocityRadPerSec.ofUnit(radians/ seconds) }
    }

    override val intakeCurrents by GroundIntakeLog.quantityList{
        intakeSims.map{ it.currentDrawAmps.ofUnit(amps) }
    }

    override val intakeTemps by GroundIntakeLog.doubleList{
        intakeSims.map{0.0}
    }

    override val conveyorCurrent by GroundIntakeLog.quantity{
        conveyorMotorSim.currentDrawAmps.ofUnit(amps)
    }

    override val conveyorSpeed by GroundIntakeLog.quantity{
        conveyorMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)
    }

    override val conveyorVoltage by GroundIntakeLog.quantity{
        _conveyorVoltage
    }

    override fun intake(voltage: Voltage) {
        _intakeVoltages[0] = voltage
        topMotorSim.setInputVoltage(voltage.inUnit(volts))
        if (bottomMotorSim != null){
            _intakeVoltages[1] = -voltage
            bottomMotorSim.setInputVoltage(-voltage.inUnit(volts))
        }
        _conveyorVoltage = voltage * conveyorVoltageMultiplier
        conveyorMotorSim.setInputVoltage(_conveyorVoltage.siValue)
    }
}