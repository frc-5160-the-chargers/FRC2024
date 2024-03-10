package frc.robot.hardware.subsystems.climber.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.CANSparkMax


class ClimberIOReal(
    private val leftMotor: CANSparkMax,
    private val rightMotor: CANSparkMax
): ClimberIO {

    private val leftEncoder = leftMotor.getEncoder()
    private val rightEncoder = rightMotor.getEncoder()


    override val leftVoltage: Voltage by ClimberLog.quantity{
        leftMotor.get() * 12.volts
    }

    override val rightVoltage: Voltage by ClimberLog.quantity{
        rightMotor.get() * 12.volts
    }

    override val leftSpeed: AngularVelocity by ClimberLog.quantity{
        leftEncoder.velocity.ofUnit(rotations / minutes)
    }
    override val rightSpeed: AngularVelocity by ClimberLog.quantity{
        rightEncoder.velocity.ofUnit(rotations / minutes)
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
