@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.wpilibextensions.kinematics.ChassisPowers

/**
 * An interface used to control differential drivetrains.
 * A differential drivetrain is one based on two sides that move independently.
 * This is similar to how a tank drives, although it must not
 * necessarily be controlled using "tank controls."
 *
 *  The Kit of Parts drivetrain each team receives is a differential drivetrain.
 *
 * See [here](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html) for more on differential drivetrains, and particularly
 * [here](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#drive-modes) for an explanation of the various drive modes.
 */
public fun interface DifferentialDrivetrain : Subsystem {
    /**
     * Drives using "tank controls", a system by which each side of the drivetrain is controlled independently.
     * @param leftPower the power of the left side of the drivetrain (from [-1..1]).
     * @param rightPower the power of the right side of the drivetrain (from [-1..1]).
     */

    public fun tankDrive(leftPower: Double, rightPower: Double)

    /**
     * Drives the robot at a certain power forward and with a certain amount of rotation.
     * @param power the power with which to drive forward (from [-1..1]).
     * @param rotation the power with which to rotate (proportional to the angular velocity, or how quickly the heading changes). (Must be from [-1..1]).
     */
    public fun arcadeDrive(power: Double, rotation: Double = 0.0){
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(power,rotation, true)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    /**
     * Drives the robot at a certain power forward and with a certain amount of steering.
     * This method makes turning easier at high speeds.
     * @param power the power with which to drive forward (from [-1..1]).
     * @param steering the amount of steering (inversely proportional to the turn radius).
     * Changing this value is can be thought of as changing how far a car's steering wheel is turned.
     * (Must be from [-1..1]).
     */
    public fun curvatureDrive(power: Double, steering: Double){
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(power,steering, true)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    /**
     * Stops the robot.
     */
    public fun stop(){
        tankDrive(0.0,0.0)
    }

    /**
     * Calls [arcadeDrive] with a [ChassisPowers].
     */
    public fun arcadeDrive(chassisPowers: ChassisPowers) {
        arcadeDrive(power = chassisPowers.xPower, rotation = chassisPowers.rotationPower)
    }

    /**
     * Calls [curvatureDrive] with a [ChassisPowers].
     */
    public fun curvatureDrive(chassisPowers: ChassisPowers) {
        curvatureDrive(power = chassisPowers.xPower, steering = chassisPowers.rotationPower)
    }
}