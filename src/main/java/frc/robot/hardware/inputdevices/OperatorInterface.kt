@file:Suppress("unused")
package frc.robot.hardware.inputdevices

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.hardware.inputdevices.withDoubleClick
import frc.robot.OPERATOR_CONTROLLER_PORT

object OperatorInterface: CommandXboxController(OPERATOR_CONTROLLER_PORT) {
    /* Top Level Constants */
    private const val SHOULD_INVERT_SHOOTER_SPEED = true
    private const val SHOULD_INVERT_PIVOT_SPEED = true

    private val keyboardNTInterface = NetworkTableInstance.getDefault().getTable("DriverStationKeyPress")

    /* Public API */
    val shooterSpeedAxis =
        InputAxis{ leftY }
            .invertWhen{ SHOULD_INVERT_SHOOTER_SPEED }
            .withDeadband(0.15)
            .withMultiplier(0.6)
            .square()

    val pivotSpeedAxis =
        InputAxis{ rightY }
            .invertWhen { SHOULD_INVERT_PIVOT_SPEED }
            .withDeadband(0.33)
            .withMultiplier(0.33)
            .square()

    fun keyPressTrigger(key: String): Trigger =
        Trigger{ keyboardNTInterface.getEntry(key).getString("") == key }

    val stowPivotTrigger: Trigger = b()

    val ampScoreTrigger: Trigger = a()
    val sourceIntakeLeftTrigger: Trigger = x()
    val sourceIntakeRightTrigger: Trigger = Trigger{ false }
    val driveToNoteTrigger: Trigger = y()

    val groundIntakeTrigger: Trigger = rightTrigger()
    val groundOuttakeTrigger: Trigger = leftTrigger()
    val passToShooterTrigger: Trigger = rightBumper()
    val shootInSpeakerTrigger: Trigger = leftBumper()

    val resetPivotAngleTrigger: Trigger = start().or(back()).withDoubleClick()
}