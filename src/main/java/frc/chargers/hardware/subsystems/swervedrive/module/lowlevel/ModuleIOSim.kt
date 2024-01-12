@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive.module.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.DEFAULT_GEAR_RATIO
import frc.chargers.constants.DEFAULT_SWERVE_DRIVE_INERTIA
import frc.chargers.constants.DEFAULT_SWERVE_TURN_INERTIA
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.inputModulus

/**
 * Represents the low level hardware of a SwerveModule during simulation.
 */
public class ModuleIOSim(
    logInputs: LoggableInputsProvider,
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
    driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA
): ModuleIO {
    private val turnMotorSim = DCMotorSim(
        turnGearbox,
        turnGearRatio,
        turnInertiaMoment.inUnit(kilo.grams * (meters * meters) )
    )

    private val driveMotorSim = DCMotorSim(
        driveGearbox,
        driveGearRatio,
        driveInertiaMoment.inUnit(kilo.grams * (meters * meters) )
    )

    private var turnAppliedVoltage = Voltage(0.0)

    private var driveAppliedVoltage = Voltage(0.0)

    init{
        ChargerRobot.runPeriodically(addToFront = true /* Makes this block run before everything else */ ){
            turnMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            driveMotorSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        }
    }

    override val logTab: String = logInputs.namespace

    override val direction: Angle by logInputs.quantity{
        turnMotorSim.angularPositionRad.ofUnit(radians).inputModulus(0.degrees..360.degrees)
    }

    override val turnSpeed: AngularVelocity by logInputs.quantity{
        turnMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val speed: AngularVelocity by logInputs.quantity{
        driveMotorSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val wheelTravel: Angle by logInputs.quantity{ driveMotorSim.angularPositionRad.ofUnit(radians) }

    override var turnVoltage: Voltage by logInputs.quantity(
        getValue = {turnAppliedVoltage},
        setValue = {
            turnAppliedVoltage = it.coerceIn((-12).volts..12.volts)
            turnMotorSim.setInputVoltage(it.inUnit(volts))
        }
    )

    override var driveVoltage: Voltage by logInputs.quantity(
        getValue = {driveAppliedVoltage},
        setValue = {
            driveAppliedVoltage = it.coerceIn((-12).volts..12.volts)
            driveMotorSim.setInputVoltage(driveAppliedVoltage.inUnit(volts))
        }
    )

    override val driveCurrent: Current by logInputs.quantity{
        driveMotorSim.currentDrawAmps.ofUnit(amps)
    }

    override val turnCurrent: Current by logInputs.quantity{
        turnMotorSim.currentDrawAmps.ofUnit(amps)
    }

    override val driveTempCelsius: Double by logInputs.double{ 0.0 }

    override val turnTempCelsius: Double by logInputs.double{ 0.0 }

}