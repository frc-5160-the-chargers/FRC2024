package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot

class PivotIOSim(private val motorSim: DCMotorSim): PivotIO {
    init{
        ChargerRobot.runPeriodically {
            motorSim.update(0.02)
        }
    }

    private var _pivotVoltage = Voltage(0.0)

    private val pivotController = getRioPIDController()

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

    override fun setPositionSetpoint(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        pivotController.constants = pidConstants
        val output = pivotController.calculateOutput(position) + ffOutput
        setVoltage(output)
    }
}