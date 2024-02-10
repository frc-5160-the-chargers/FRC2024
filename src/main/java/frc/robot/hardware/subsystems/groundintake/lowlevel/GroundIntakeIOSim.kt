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
    private vararg val motorSims: DCMotorSim
): GroundIntakeIO {
    // must be array to allow mutability but no adding values
    private val _appliedVoltages = motorSims.map{ Voltage(0.0) }.toTypedArray()

    init{
        ChargerRobot.runPeriodically(addToFront = true) {
            motorSims.forEachIndexed{index, motorSim ->
                motorSim.update(0.02)
                motorSim.setInputVoltage(_appliedVoltages[index].inUnit(volts))
            }
        }
    }

    override val measuredVoltages by GroundIntakeLog.quantityList{
        _appliedVoltages.toList()
    }
    override val measuredSpeeds by GroundIntakeLog.quantityList{
        motorSims.map{ it.angularVelocityRadPerSec.ofUnit(radians/ seconds) }
    }
    override val measuredCurrents by GroundIntakeLog.quantityList{
        motorSims.map{ it.currentDrawAmps.ofUnit(amps) }
    }

    override fun intake(voltage: Voltage) {
        _appliedVoltages[0] = voltage
        if (_appliedVoltages.size > 1){
            _appliedVoltages[1] = -voltage
        }
    }
}