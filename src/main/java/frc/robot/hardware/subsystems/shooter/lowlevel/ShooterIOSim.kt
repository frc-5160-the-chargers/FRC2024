package frc.robot.hardware.subsystems.shooter.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot

@Suppress("unused")
class ShooterIOSim(
    private val intakeSims: List<DCMotorSim>,
    private val pivotSim: DCMotorSim
): ShooterIO {
    private var _intakeVoltages: Array<Voltage> = intakeSims.map{ Voltage(0.0) }.toTypedArray() // 1 voltage value per sim
    private var _pivotVoltage = Voltage(0.0)

    private val pivotController = SuperPIDController(
        PIDConstants(0.3,0.0,0.0),
        getInput = { pivotPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )

    init{
        ChargerRobot.runPeriodically(addToFront = true) {
            intakeSims.forEachIndexed{ index, motorSim ->
                motorSim.update(0.02)
                motorSim.setInputVoltage(intakeVoltages[index].inUnit(volts))
            }
            pivotSim.update(0.02)
        }
    }


    override val hasGamepiece by ShooterLog.boolean{ false }
    override val hasBeamBreakSensor by ShooterLog.boolean{ false }

    override val intakeVoltages by ShooterLog.quantityList {
        _intakeVoltages.toList()
    }
    override val intakeSpeeds by ShooterLog.quantityList {
        intakeSims.map{ it.angularVelocityRadPerSec.ofUnit(radians/seconds) }
    }
    override val intakeCurrents by ShooterLog.quantityList {
        intakeSims.map{ it.currentDrawAmps.ofUnit(amps) }
    }
    override val intakeTemps by ShooterLog.doubleList {
        intakeSims.map{ 0.0 }
    }


    override fun setVoltage(voltage: Voltage) {
        _intakeVoltages[0] = voltage
        intakeSims[0].setInputVoltage(voltage.inUnit(volts))
    }


    override val pivotVoltage by ShooterLog.quantity { _pivotVoltage }
    override val pivotPosition by ShooterLog.quantity{ pivotSim.angularPositionRad.ofUnit(radians) }
    override val pivotCurrent by ShooterLog.quantity{ pivotSim.currentDrawAmps.ofUnit(amps) }
    override val pivotTemp by ShooterLog.double{ 0.0 }


    override fun setPivotVoltage(voltage: Voltage) {
        pivotSim.setInputVoltage(voltage.inUnit(volts))
    }

    override fun setPivotPosition(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        pivotController.constants = pidConstants
        pivotController.target = position
        setPivotVoltage(pivotController.calculateOutput() + ffOutput)
    }
}