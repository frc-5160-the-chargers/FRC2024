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


    val driveToAmpTrigger: Trigger = a()
    val driveToSourceLeftTrigger: Trigger = povLeft()
    val driveToSourceRightTrigger: Trigger = povRight()

    val aimToSpeakerTrigger: Trigger = b()

    val groundIntakeTrigger: Trigger = rightTrigger()
    val groundOuttakeTrigger: Trigger = leftTrigger()
}