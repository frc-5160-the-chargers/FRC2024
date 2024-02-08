package frc.robot.hardware.subsystems.groundintake

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.constants.DEFAULT_GEAR_RATIO
import frc.chargers.framework.ChargerRobot

@Suppress("unused")
class GroundIntakeIOSim(
    topSimMotor: DCMotor,
    gearRatio: Double = DEFAULT_GEAR_RATIO
): GroundIntakeIO {
    private val motorSims = listOf(
        DCMotorSim(topSimMotor, gearRatio, 0.04)
    )

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

    override fun setVoltage(voltage: Voltage) {
        _appliedVoltages[0] = voltage
    }
}