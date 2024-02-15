package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot

class PivotIOSim(private val motorSim: DCMotorSim): PivotIO {
    init{
        ChargerRobot.runPeriodically {
            motorSim.update(0.02)
        }
    }

    private var _pivotVoltage = Voltage(0.0)

    private val pivotController = SuperPIDController(
        PIDConstants(0.3,0.0,0.0),
        getInput = { position },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )

    override val appliedVoltage by PivotLog.quantity {
        _pivotVoltage
    }

    override val position by PivotLog.quantity{
        motorSim.angularPositionRad.ofUnit(radians)
    }

    override val appliedCurrent by PivotLog.quantity{
        motorSim.currentDrawAmps.ofUnit(amps)
    }

    override val tempCelsius by PivotLog.double{
        0.0
    }

    override fun setVoltage(voltage: Voltage) {
        _pivotVoltage = voltage
        motorSim.setInputVoltage(voltage.siValue)
    }

    override fun setPosition(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        pivotController.constants = pidConstants
        pivotController.target = position
        val output = pivotController.calculateOutput() + ffOutput
        setVoltage(output)
    }
}