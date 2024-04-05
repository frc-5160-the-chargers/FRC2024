@file:Suppress("unused")
package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.hardware.inputdevices.withDoubleClick
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.DRIVER_CONTROLLER_PORT
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean


object DriverController: CommandPS5Controller(DRIVER_CONTROLLER_PORT){
    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.1
    private val DRIVER = Driver.NAYAN

    private val shouldInvertForward = LoggedDashboardBoolean("ShouldInvertForward", false)
    private val shouldInvertStrafe = LoggedDashboardBoolean("ShouldInvertStrafe", false)



    /* Public API */
    val pointNorthTrigger: Trigger = if (DRIVER.rightHanded) povUp() else triangle()
    val pointSouthTrigger: Trigger = if (DRIVER.rightHanded) povDown() else cross()
    val pointEastTrigger: Trigger = if (DRIVER.rightHanded) povRight() else circle()
    val pointWestTrigger: Trigger = if (DRIVER.rightHanded) povLeft() else square()

    val driveToNoteAssistTrigger: Trigger = if (DRIVER.rightHanded) L2() else R2()

    val aimToSpeakerTrigger: Trigger = Trigger{ false }

    val climbersUpTrigger: Trigger = povUp()
    val climbersDownTrigger: Trigger = povDown()
    val zeroHeadingTrigger: Trigger = touchpad().withDoubleClick()

    val shouldDisableFieldRelative: Boolean
        get() = touchpad().asBoolean


    /* Private implementation */
    private val forwardAxis =
        InputAxis{ if (DRIVER.rightHanded) rightY else leftY }
            .withDeadband(DEFAULT_DEADBAND)
            .invertWhen(RobotBase::isReal)
            .invertWhen(shouldInvertForward::get)
            .log("DriverController/xPower")

    private val strafeAxis =
        InputAxis{ if (DRIVER.rightHanded) rightX else leftX }
            .withDeadband(DEFAULT_DEADBAND)
            .invertWhen(RobotBase::isReal)
            .invertWhen(shouldInvertStrafe::get)
            .log("DriverController/yPower")

    private val rotationAxis =
        InputAxis{ if (DRIVER.rightHanded) leftX else rightX }
            .withDeadband(DEFAULT_DEADBAND)
            .square()
            .withEquation(Polynomial(-0.2,0.0,-0.5,0.0))
            .log("DriverController/rotationPower")

    private val precisionAxis =
        InputAxis{ if (DRIVER.rightHanded) r2Axis else l2Axis }
            .mapToRange(1.0..7.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .withModifier{ 1.0 / it }
            .log("DriverController/precisionPower")


    val swerveOutput: ChassisPowers get(){
        val scalar = precisionAxis()

        return ChassisPowers(
            xPower = forwardAxis() * scalar,
            yPower = strafeAxis() * scalar,
            rotationPower = rotationAxis() * scalar
        )
    }
}