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
            .withDeadband(0.15)
            .invertWhen{ SHOULD_INVERT_SHOOTER_SPEED }
            .withMultiplier(0.8)
            .log("Shooter/outtakingSpeed")

    val pivotSpeedAxis =
        InputAxis{ rightY }
            .withDeadband(0.33)
            .invertWhen { SHOULD_INVERT_PIVOT_SPEED }
            .withMultiplier(0.33)
            .square()
            .log("Shooter/pivotSpeed")

    fun keyPressTrigger(key: String): Trigger =
        Trigger{ keyboardNTInterface.getEntry(key).getString("") == key }

    val stowPivotTrigger: Trigger = b()

    val ampPositionTrigger: Trigger = a()
    val sourcePositionTrigger: Trigger = x()
    val quickScoreAmpTrigger: Trigger = y()

    val groundIntakeTrigger: Trigger = rightTrigger()
    val groundOuttakeTrigger: Trigger = leftTrigger()

    val passToShooterTrigger: Trigger = rightBumper().and(leftBumper().negate())
    val spinUpShooterTrigger: Trigger = leftBumper().and(rightBumper().negate())
    val shootInSpeakerTrigger: Trigger = leftBumper().and(rightBumper())

    val resetPivotAngleTrigger: Trigger = start().or(back()).withDoubleClick()
}