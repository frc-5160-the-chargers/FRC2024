@file:Suppress("unused")
package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.DRIVER_CONTROLLER_PORT
import kotlin.jvm.optionals.getOrNull


object DriverController: CommandXboxController(DRIVER_CONTROLLER_PORT){
    enum class Driver(val rightHanded: Boolean) {
        NAYAN(false),
        KENNA(true),
        CONRAD(true),
        JOYCE(true),
        JACK(true)
    }

    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.2
    private const val IS_KEYBOARD_SIM_CONTROLLER = false
    private val DRIVER = Driver.NAYAN


    /* Public API */
    val pointNorthTrigger: Trigger = if (DRIVER.rightHanded) povUp() else y()
    val pointSouthTrigger: Trigger = if (DRIVER.rightHanded) povDown() else a()
    val pointEastTrigger: Trigger = if (DRIVER.rightHanded) povRight() else x()
    val pointWestTrigger: Trigger = if (DRIVER.rightHanded) povLeft() else b()

    val aimToSpeakerTrigger: Trigger = Trigger{ false }
    val climberUpTrigger: Trigger = rightBumper()
    val climberDownTrigger: Trigger = leftBumper()

    val shouldDisableFieldRelative: Boolean
        get() = start().asBoolean || back().asBoolean


    /* Private implementation */
    private val forwardAxis =
        InputAxis{ if (DRIVER.rightHanded) rightY else leftY }
            .applyDeadband(DEFAULT_DEADBAND)
            .invertWhen{
                (DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red &&
                    !IS_KEYBOARD_SIM_CONTROLLER) ||
                    shouldDisableFieldRelative
            }
            .applyMultiplier(0.6)
            .log("DriverController/xPower")

    private val strafeAxis =
        InputAxis{ if (DRIVER.rightHanded) rightX else leftX }
            .applyDeadband(0.3)
            .invertWhen{
                (DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red &&
                    !IS_KEYBOARD_SIM_CONTROLLER) ||
                    shouldDisableFieldRelative
            }
            .applyMultiplier(0.6)
            .log("DriverController/yPower")

    private val rotationAxis =
        InputAxis{ if (DRIVER.rightHanded) leftX else rightX }
            .applyDeadband(DEFAULT_DEADBAND)
            .square()
            .applyEquation(Polynomial(-0.1,0.0,-0.4,0.0))
            .log("DriverController/rotationPower")


    private val turboAxis =
        InputAxis{ rightTriggerAxis }
            .mapToRange(1.0..2.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .log("DriverController/turboPower")

    private val precisionAxis =
        InputAxis{ leftTriggerAxis }
            .mapToRange(1.0..4.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .withModifier{ 1.0 / it }
            .log("DriverController/precisionPower")



    val swerveOutput: ChassisPowers get(){
        val scalar = turboAxis() * precisionAxis()

        return ChassisPowers(
            xPower = forwardAxis() * scalar,
            yPower = strafeAxis() * scalar,
            rotationPower = rotationAxis() * scalar
        )
    }
}