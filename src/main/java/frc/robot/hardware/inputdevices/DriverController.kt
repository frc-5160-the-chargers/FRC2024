@file:Suppress("unused")
package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.hardware.inputdevices.withDoubleClick
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.DRIVER_CONTROLLER_PORT
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean


object DriverController: CommandXboxController(DRIVER_CONTROLLER_PORT){
    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.1
    private val DRIVER = Driver.NAYAN

    private val shouldInvertForward = LoggedDashboardBoolean("ShouldInvertForward", false)
    private val shouldInvertStrafe = LoggedDashboardBoolean("ShouldInvertStrafe", false)


    /* Public API */
    val pointNorthTrigger: Trigger = if (DRIVER.rightHanded) povUp() else y()
    val pointSouthTrigger: Trigger = if (DRIVER.rightHanded) povDown() else a()
    val pointEastTrigger: Trigger = if (DRIVER.rightHanded) povRight() else x()
    val pointWestTrigger: Trigger = if (DRIVER.rightHanded) povLeft() else b()

    val aimToSpeakerTrigger: Trigger = Trigger{ false }


    /*
    val leftHookUpTrigger: Trigger = leftBumper().and(
        if (DRIVER.rightHanded){
            y()
        }else{
            povUp()
        }
    )

    val leftHookDownTrigger: Trigger = leftBumper().and(
        if (DRIVER.rightHanded){
            a()
        }else{
            povDown()
        }
    )

    val rightHookUpTrigger: Trigger = rightBumper().and(
        if (DRIVER.rightHanded) y() else povUp()
    )

    val rightHookDownTrigger: Trigger = rightBumper().and(
        if (DRIVER.rightHanded) a() else povDown()
    )

     */

    val climbersUpTrigger: Trigger = povUp()
    val climbersDownTrigger: Trigger = povDown()
    val zeroHeadingTrigger: Trigger = start().or(back()).withDoubleClick()

    val shouldDisableFieldRelative: Boolean
        get() = start().asBoolean || back().asBoolean


    /* Private implementation */
    private val forwardAxis =
        InputAxis{ if (DRIVER.rightHanded) rightY else leftY }
            .applyDeadband(DEFAULT_DEADBAND)
            .invert()
            //.invertWhen{ shouldInvertForward.get() }
            .applyMultiplier(0.6)
            .log("DriverController/xPower")

    private val strafeAxis =
        InputAxis{ if (DRIVER.rightHanded) rightX else leftX }
            .applyDeadband(DEFAULT_DEADBAND)
            .invert()
            //.invertWhen{ shouldInvertStrafe.get() }
            .applyMultiplier(0.6)
            .log("DriverController/yPower")

    private val rotationAxis =
        InputAxis{ if (DRIVER.rightHanded) leftX else rightX }
            .applyDeadband(DEFAULT_DEADBAND)
            /*
            .invertWhen{
                (DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red &&
                        !IS_KEYBOARD_SIM_CONTROLLER) ||
                        shouldDisableFieldRelative
            }
             */
            .square()
            .applyEquation(Polynomial(0.1,0.0,0.4,0.0))
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