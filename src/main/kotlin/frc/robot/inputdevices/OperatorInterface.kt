package frc.robot.inputdevices

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.framework.Loggable
import frc.chargers.wpilibextensions.inputdevices.withDoubleClick
import frc.chargers.utils.math.withDeadband
import frc.robot.OPERATOR_CONTROLLER_PORT
import kotlin.math.pow

@Suppress("unused")
object OperatorInterface: CommandXboxController(OPERATOR_CONTROLLER_PORT), Loggable {
    override val namespace = "OperatorInterface"
    /* Top Level Constants */
    private const val SHOULD_INVERT_SHOOTER_SPEED = true
    private const val SHOULD_INVERT_PIVOT_SPEED = true

    private val keyboardNTInterface = NetworkTableInstance.getDefault().getTable("DriverStationKeyPress")

    /* Public API */
    val shooterSpeed: Double by logged{
        var speed = leftY.withDeadband(0.15)
        if (SHOULD_INVERT_SHOOTER_SPEED) speed *= -1
        speed *= 0.8
        return@logged speed
    }

    val pivotSpeed: Double by logged{
        var speed = rightY.withDeadband(0.33)
        if (SHOULD_INVERT_PIVOT_SPEED) speed *= -1
        speed = (0.33 * speed).pow(2)
        return@logged speed
    }

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