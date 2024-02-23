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

    val stowPivotTrigger: Trigger = b()
    val driveToAmpTrigger: Trigger = a()
    val driveToSourceLeftTrigger: Trigger = x()
    val driveToSourceRightTrigger: Trigger = y()

    val aimToSpeakerTrigger: Trigger = start().or(back())

    val groundIntakeTrigger: Trigger = rightTrigger().and(leftTrigger().negate())
    val groundOuttakeTrigger: Trigger = leftTrigger().and(rightTrigger().negate())
    val passToShooterTrigger: Trigger = leftTrigger().and(rightTrigger())

}