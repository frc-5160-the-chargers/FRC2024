package frc.robot.rigatoni.inputdevices

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.chargers.framework.Loggable
import frc.chargers.utils.mapControllerInput
import frc.chargers.utils.squareMagnitude
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.abs
import kotlin.math.pow

class PS5SwerveController(
    port: Int,
    private val defaultDeadband: Double,
    private val driverRightHanded: Boolean
): CommandPS5Controller(port), Loggable {
    override val namespace = "RobotGeneral/DriveController"

    private fun filterNan(input: Double): Double =
        if (input.isInfinite() || input.isNaN()) 0.0 else input

    private fun rotationEquation(x: Double): Double {
        return -0.2 * x.pow(3) - 0.5 * x
    }

    private var forward by logged(0.0)
    private var strafe by logged(0.0)
    private var rotation by logged(0.0)
    private var scalar by logged(0.0, "OutputMultiplier")

    val swerveOutput: ChassisPowers get() {
        forward = filterNan(if (driverRightHanded) rightY else leftY)
        forward = applyDeadband(forward, defaultDeadband)
        if (RobotBase.isReal()) forward *= -1

        strafe = filterNan(if (driverRightHanded) rightX else leftX)
        strafe = applyDeadband(strafe, defaultDeadband)
        if (RobotBase.isReal()) strafe *= -1

        rotation = filterNan(if (driverRightHanded) leftX else rightX)
        rotation = applyDeadband(rotation, defaultDeadband).squareMagnitude()
        rotation = rotationEquation(rotation)

        scalar = abs(filterNan(if (driverRightHanded) r2Axis else l2Axis))
        scalar = 1.0 / abs(scalar).mapControllerInput(1.0..7.0)

        return ChassisPowers(
            xPower = forward * scalar,
            yPower = strafe * scalar,
            rotationPower = rotation * scalar
        )
    }
}