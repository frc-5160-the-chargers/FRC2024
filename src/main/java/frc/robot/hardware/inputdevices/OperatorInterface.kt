@file:Suppress("unused")
package frc.robot.hardware.inputdevices

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.OPERATOR_CONTROLLER_PORT

object OperatorInterface: CommandXboxController(OPERATOR_CONTROLLER_PORT) {
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