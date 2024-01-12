@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase


/**
 * A simple implementation of a [DifferentialDrivetrain].
 *
 * @see DifferentialDrivetrain
 */
public open class BasicDifferentialDrivetrain(
    leftMotors: MotorController,
    rightMotors: MotorController,
    invertMotors: Boolean = false,
    protected val powerScale: Double = 1.0,
    protected val rotationScale: Double = 1.0,
) : SubsystemBase(), DifferentialDrivetrain {
    protected val differentialDrive: DifferentialDrive = DifferentialDrive(leftMotors, rightMotors)

    init {
        leftMotors.inverted = false
        rightMotors.inverted = true

        if (invertMotors) {
            leftMotors.inverted = !leftMotors.inverted
            rightMotors.inverted = !rightMotors.inverted
        }
    }

    override fun tankDrive(leftPower: Double, rightPower: Double) {
        differentialDrive.tankDrive(
            leftPower * powerScale,
            rightPower * powerScale,
            true
        )
    }

    override fun arcadeDrive(power: Double, rotation: Double) {
        differentialDrive.arcadeDrive(
            power * powerScale,
            rotation * rotationScale,
            false
        )
    }

    public override fun curvatureDrive(power: Double, steering: Double) {
        differentialDrive.curvatureDrive(
            power * powerScale,
            steering * rotationScale,
            true
        )
    }

    override fun stop() {
        differentialDrive.stopMotor()
    }
}