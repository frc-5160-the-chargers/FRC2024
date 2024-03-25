package frc.robot.hardware.subsystems.climber.lowlevel

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.volts
import com.revrobotics.CANSparkMax

/**
 * The real component of the climber
 *
 * pre-requirements: Climber hooks are at the top of the robot.
 */
class ClimberIOReal(
    private val leftMotor: CANSparkMax,
    private val rightMotor: CANSparkMax,
): ClimberIO {

    private val leftEncoder = leftMotor.getEncoder()
    private val rightEncoder = rightMotor.getEncoder()

    init{
        // sets position to 0.0
        leftEncoder.setPosition(0.0)
        rightEncoder.setPosition(0.0)
    }


    override val leftVoltage: Voltage by ClimberLog.quantity{
        leftMotor.get() * 12.volts
    }

    override val rightVoltage: Voltage by ClimberLog.quantity{
        rightMotor.get() * 12.volts
    }

    override val leftPosition by ClimberLog.quantity{
        leftEncoder.position.ofUnit(rotations)
    }

    override val rightPosition by ClimberLog.quantity{
        rightEncoder.position.ofUnit(rotations)
    }

    override fun setLeftVoltage(voltage: Voltage) {
        leftMotor.set(voltage.siValue / 12.0)
    }

    override fun setRightVoltage(voltage: Voltage) {
        rightMotor.set(voltage.siValue / 12.0)
    }
}
