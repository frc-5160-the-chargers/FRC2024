@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.wpilibextensions.kinematics.ChassisPowers


public fun DifferentialDrivetrain(
    leftMotors: List<MotorController>,
    rightMotors: List<MotorController>,
    invertMotors: Boolean = false
) = DifferentialDrivetrain{ leftPower, rightPower ->
    val scalar = if (invertMotors) -1.0 else 1.0
    leftMotors.forEach { it.set(leftPower * scalar) }
    rightMotors.forEach { it.set(rightPower * scalar) }
}

/**
 * A interface that controls a differential drivetrain.
 *
 * This interface is a functional interface; which means that you can instantiate an anonymous implementation of it
 * to easily represent any kind of differential drive.
 *
 * For instance:
 * ```
 * val drivetrain = DifferentialDrivetrain{ leftPower, rightPower -> leftMotors.set(leftPower); rightMotors.set(rightPower) }
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
    public fun arcadeDrive(power: Double, rotation: Double = 0.0, squareInputs: Boolean = true){
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(power,rotation, squareInputs)
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
    public fun curvatureDrive(power: Double, steering: Double, allowTurnInPlace: Boolean = true){
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(power,steering, allowTurnInPlace)
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
    public fun arcadeDrive(chassisPowers: ChassisPowers, squareInputs: Boolean = true) {
        arcadeDrive(power = chassisPowers.xPower, rotation = chassisPowers.rotationPower, squareInputs)
    }

    /**
     * Calls [curvatureDrive] with a [ChassisPowers].
     */
    public fun curvatureDrive(chassisPowers: ChassisPowers, allowTurnInPlace: Boolean = true) {
        curvatureDrive(power = chassisPowers.xPower, steering = chassisPowers.rotationPower, allowTurnInPlace)
    }
}