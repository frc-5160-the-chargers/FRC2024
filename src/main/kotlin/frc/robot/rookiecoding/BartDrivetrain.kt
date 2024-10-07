package frc.robot.rookiecoding

import edu.wpi.first.wpilibj.drive.DifferentialDrive

/**
 * TODO: finish this drivetrain class.
 *
 * Components:
 * - Extend SubsystemBase
 * - Provide drive methods
 *
 * Motors:
 * All spark maxes(let the IDs be whatever you want)
 */
class BartDrivetrain {
    fun tankDrive(leftPower: Double, rightPower: Double) {
        // TODO: Finish this
    }
    fun arcadeDrive(xPower: Double, rotationPower: Double) {
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(xPower, rotationPower, true)
        tankDrive(wheelSpeeds.left, wheelSpeeds.right)
    }
}