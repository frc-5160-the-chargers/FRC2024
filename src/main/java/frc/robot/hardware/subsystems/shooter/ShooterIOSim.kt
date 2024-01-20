package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot

@Suppress("unused")
class ShooterIOSim(
    leftSimMotors: DCMotor,
    rightSimMotors: DCMotor,
    pivotMotors: DCMotor,

    leftGearRatio: Double,
    rightGearRatio: Double,
    pivotGearRatio: Double
): ShooterIO {

    private val leftIntakeSim = DCMotorSim(leftSimMotors, leftGearRatio, 0.004)
    private val rightIntakeSim = DCMotorSim(rightSimMotors, rightGearRatio, 0.004)

    private val pivotSim = DCMotorSim(pivotMotors, pivotGearRatio, 0.010)

    private var _leftIntakeVoltage = Voltage(0.0)
    private var _rightIntakeVoltage = Voltage(0.0)
    private var _pivotVoltage = Voltage(0.0)

    private val pivotController = SuperPIDController(
        PIDConstants(0.3,0.0,0.0),
        getInput = { pivotPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )

    init{
        ChargerRobot.runPeriodically( addToFront = true ) {
            leftIntakeSim.update(0.02)
            rightIntakeSim.update(0.02)
            pivotSim.update(0.02)
        }
    }


    override val hasGamepiece by ShooterLog.boolean{ false }
    override val hasBeamBreakSensor by ShooterLog.boolean{ false }

    override val leftIntakeVoltage by ShooterLog.quantity { _leftIntakeVoltage }
    override val rightIntakeVoltage by ShooterLog.quantity { _rightIntakeVoltage }

    override val leftIntakeSpeed by ShooterLog.quantity{
        leftIntakeSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }
    override val rightIntakeSpeed by ShooterLog.quantity{
        rightIntakeSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override val leftIntakeCurrent by ShooterLog.quantity{
        leftIntakeSim.currentDrawAmps.ofUnit(amps)
    }
    override val rightIntakeCurrent by ShooterLog.quantity{
        rightIntakeSim.currentDrawAmps.ofUnit(amps)
    }

    override val leftIntakeTemp by ShooterLog.double{0.0}
    override val rightIntakeTemp by ShooterLog.double{0.0}


    override fun spin(leftVoltage: Voltage, rightVoltage: Voltage) {
        _leftIntakeVoltage = leftVoltage
        _rightIntakeVoltage = rightVoltage
        leftIntakeSim.setInputVoltage(_leftIntakeVoltage.inUnit(volts))
        rightIntakeSim.setInputVoltage(_rightIntakeVoltage.inUnit(volts))
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