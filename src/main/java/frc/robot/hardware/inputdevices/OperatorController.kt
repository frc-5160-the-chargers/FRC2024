package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.constants.OPERATOR_CONTROLLER_PORT

object OperatorController: CommandXboxController(OPERATOR_CONTROLLER_PORT) {
    val aimToTagButton: Trigger = a()
    val aimToTagAndDriveButton: Trigger = b()
}