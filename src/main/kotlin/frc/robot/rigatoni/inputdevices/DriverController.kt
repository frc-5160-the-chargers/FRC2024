@file:Suppress("unused")
package frc.robot.rigatoni.inputdevices

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.framework.Loggable
import frc.chargers.wpilibextensions.inputdevices.withDoubleClick
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.mapAxisTo
import frc.chargers.utils.math.squareMagnitude
import frc.chargers.utils.math.withDeadband
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.abs


object DriverController: CommandPS5Controller(DRIVER_CONTROLLER_PORT), Loggable {
    override val namespace = "DriverController"

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


    private fun filterNan(input: Double): Double =
        if (input.isInfinite() || input.isNaN()) 0.0 else input

    private val rotationEquation = Polynomial(-0.2,0.0,-0.5,0.0)

    private var forward by logged(0.0)
    private var strafe by logged(0.0)
    private var rotation by logged(0.0)
    private var scalar by logged(0.0, "OutputScalar")


    val swerveOutput: ChassisPowers get(){
        forward = filterNan(if (DRIVER.rightHanded) rightY else leftY)
        forward = forward.withDeadband(DEFAULT_DEADBAND)
        if (RobotBase.isReal()) forward *= -1

        strafe = filterNan(if (DRIVER.rightHanded) rightX else leftX)
        strafe = strafe.withDeadband(DEFAULT_DEADBAND)
        if (RobotBase.isReal()) strafe *= -1

        rotation = filterNan(if (DRIVER.rightHanded) leftX else rightX)
        rotation = rotation.withDeadband(DEFAULT_DEADBAND).squareMagnitude()
        rotation = rotationEquation(rotation)

        scalar = abs(filterNan(if (DRIVER.rightHanded) r2Axis else l2Axis))
        scalar = 1.0 / abs(scalar).mapAxisTo(1.0..7.0)

        return ChassisPowers(
            xPower = forward * scalar,
            yPower = strafe * scalar,
            rotationPower = rotation * scalar
        )
    }
}