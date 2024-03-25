package frc.robot.hardware.subsystems.climber.lowlevel

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.framework.ChargerRobot

class ClimberIOSim(
    private val leftMotorSim: DCMotorSim,
    private val rightMotorSim: DCMotorSim
): ClimberIO {

    init{
        ChargerRobot.runPeriodically{
            leftMotorSim.update(0.02)
            rightMotorSim.update(0.02)
        }
    }

    override val leftVoltage: Voltage by ClimberLog.quantity{
        0.volts
    }

    override val rightVoltage: Voltage by ClimberLog.quantity {
        0.volts
    }

    override val leftPosition by ClimberLog.quantity{
        leftMotorSim.angularPositionRotations.ofUnit(rotations)
    }

    override val rightPosition by ClimberLog.quantity{
        rightMotorSim.angularPositionRotations.ofUnit(rotations)
    }

    override fun setLeftVoltage(voltage: Voltage) {
        leftMotorSim.setInputVoltage(voltage.inUnit(volts))
    }

    override fun setRightVoltage(voltage: Voltage) {
        rightMotorSim.setInputVoltage(voltage.inUnit(volts))
    }
}