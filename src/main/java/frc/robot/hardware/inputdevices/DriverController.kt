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
    enum class Driver {
        NAYAN, KENNA, CONRAD
    }

    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.2
    private const val IS_KEYBOARD_SIM_CONTROLLER = true
    private val DRIVER = Driver.NAYAN


    /* Public API */
    val pointNorthTrigger: Trigger = when(DRIVER){
        Driver.NAYAN -> y()
        else -> povUp()
    }
    val pointSouthTrigger: Trigger = when(DRIVER){
        Driver.NAYAN -> a()
        else -> povDown()
    }
    val pointEastTrigger: Trigger = when(DRIVER){
        Driver.NAYAN -> x()
        else -> povRight()
    }
    val pointWestTrigger: Trigger = when(DRIVER){
        Driver.NAYAN -> b()
        else -> povLeft()
    }

    val shouldDisableFieldRelative: Boolean
        get() = start().asBoolean || back().asBoolean


    /* Private implementation */
    private val forwardAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> leftY

                Driver.KENNA, Driver.CONRAD -> rightY
            }
        }
            .applyDeadband(DEFAULT_DEADBAND)
            .invertWhen{
                DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red &&
                    !IS_KEYBOARD_SIM_CONTROLLER &&
                    !shouldDisableFieldRelative
            }
            .applyMultiplier(0.6)
            .log("DriverController/xPower")

    private val strafeAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> leftX

                Driver.KENNA, Driver.CONRAD -> rightX
            }
        }
            .applyDeadband(0.3)
            .invertWhen{
                DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red &&
                        !IS_KEYBOARD_SIM_CONTROLLER &&
                        !shouldDisableFieldRelative
            }
            .applyMultiplier(0.6)
            .log("DriverController/yPower")

    private val rotationAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> rightX

                Driver.KENNA, Driver.CONRAD -> leftX
            }
        }
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