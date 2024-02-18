package frc.chargers.hardware.subsystems.swervedrive.modulelowlevel

import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.inputModulus

class ModuleIOSim(
    logInputs: LoggableInputsProvider,
    private val turnMotorSim: DCMotorSim,
    private val driveMotorSim: DCMotorSim
): ModuleIO() {
    private var turnAppliedVoltage = Voltage(0.0)
    private var driveAppliedVoltage = Voltage(0.0)

    init{
        ChargerRobot.runPeriodically(addToFront = true /* Makes this block run before everything else */ ){
            turnMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            driveMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        }
    }

    override val direction: Angle by logInputs.quantity{
        turnMotorSim.angularPositionRad.ofUnit(radians).inputModulus(0.degrees..360.degrees)
    }

    override val turnSpeed: AngularVelocity by logInputs.quantity{
        turnMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val speed: AngularVelocity by logInputs.quantity{
        driveMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val wheelTravel: Angle by logInputs.quantity{
        driveMotorSim.angularPositionRad.ofUnit(radians)
    }

    override val turnVoltage: Voltage by logInputs.quantity{
        turnAppliedVoltage
    }

    override val driveVoltage: Voltage by logInputs.quantity{
        driveAppliedVoltage
    }

    override val driveCurrent: Current by logInputs.quantity{
        driveMotorSim.currentDrawAmps.ofUnit(amps)
    }

    override val turnCurrent: Current by logInputs.quantity{
        turnMotorSim.currentDrawAmps.ofUnit(amps)
    }

    override fun setTurnVoltage(voltage: Voltage) {
        turnAppliedVoltage = voltage.coerceIn((-12).volts..12.volts)
        turnMotorSim.setInputVoltage(turnAppliedVoltage.inUnit(volts))
    }

    override fun setDriveVoltage(voltage: Voltage) {
        driveAppliedVoltage = voltage.coerceIn((-12).volts..12.volts)
        driveMotorSim.setInputVoltage(driveAppliedVoltage.inUnit(volts))
    }
}