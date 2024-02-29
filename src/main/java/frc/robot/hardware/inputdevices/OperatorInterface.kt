@file:Suppress("unused")
package frc.robot.hardware.inputdevices

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.OPERATOR_CONTROLLER_PORT

object OperatorInterface: CommandXboxController(OPERATOR_CONTROLLER_PORT) {
    private const val SHOULD_INVERT_SHOOTER_SPEED = true
    private const val SHOULD_INVERT_PIVOT_SPEED = true

    val shooterSpeed: Double
        get() = leftY * if(SHOULD_INVERT_SHOOTER_SPEED) -1.0 else 1.0

    val pivotSpeed: Double
        get() = rightY / 5.0 * if (SHOULD_INVERT_PIVOT_SPEED) -1.0 else 1.0



    private val keyboardNTInterface = NetworkTableInstance.getDefault().getTable("DriverStationKeyPress")
    fun keyPressTrigger(key: String): Trigger =
        Trigger{ keyboardNTInterface.getEntry(key).getString("") == key }


    val stowPivotTrigger: Trigger = Trigger{false}
    val driveToAmpTrigger: Trigger = a()
    val driveToSourceLeftTrigger: Trigger = x()
    val driveToSourceRightTrigger: Trigger = y()

    val aimToSpeakerTrigger: Trigger = b() // tbd; might want driver to aim to speaker

    val groundIntakeTrigger: Trigger = rightTrigger()
    val groundOuttakeTrigger: Trigger = leftTrigger()
    val passToShooterTrigger: Trigger = rightBumper()
    val shootInSpeakerTrigger: Trigger = leftBumper()

    val climberUpTrigger: Trigger = povUp()
    val climberDownTrigger: Trigger = povDown()
}