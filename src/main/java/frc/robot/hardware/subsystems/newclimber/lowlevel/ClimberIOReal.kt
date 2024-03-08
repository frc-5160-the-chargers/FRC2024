package frc.robot.hardware.subsystems.newclimber.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.revrobotics.CANSparkMax


class ClimberIOReal(
    private val leftMotor: CANSparkMax,
    private val rightMotor: CANSparkMax
): ClimberIO {

    private val leftEncoder = leftMotor.getEncoder()
    private val rightEncoder = rightMotor.getEncoder()


    override val leftVoltage: Voltage by frc.robot.hardware.subsystems.climber.lowlevel.ClimberLog.quantity{
        leftMotor.get() * 12.volts
    }

    override val rightVoltage: Voltage by frc.robot.hardware.subsystems.climber.lowlevel.ClimberLog.quantity{
        rightMotor.get() * 12.volts
    }

    override val leftSpeed: AngularVelocity by frc.robot.hardware.subsystems.climber.lowlevel.ClimberLog.quantity{
        leftEncoder.velocity.ofUnit(rotations / minutes)
    }
    override val rightSpeed: AngularVelocity by frc.robot.hardware.subsystems.climber.lowlevel.ClimberLog.quantity{
        rightEncoder.velocity.ofUnit(rotations / minutes)
    }

    override val leftPosition by frc.robot.hardware.subsystems.climber.lowlevel.ClimberLog.quantity{
        leftEncoder.position.ofUnit(rotations)
    }

    override val rightPosition by frc.robot.hardware.subsystems.climber.lowlevel.ClimberLog.quantity{
        rightEncoder.position.ofUnit(rotations)
    }

    override fun setLeftVoltage(voltage: Voltage) {
        leftMotor.setVoltage(leftVoltage.inUnit(volts))
    }

    override fun setRightVoltage(voltage: Voltage) {
        rightMotor.setVoltage(rightVoltage.inUnit(volts))
    }
}
