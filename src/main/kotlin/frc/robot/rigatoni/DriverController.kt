package frc.robot.rigatoni

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.chargers.framework.logged
import frc.chargers.framework.tunable
import frc.chargers.utils.squareMagnitude
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.pow

class DriverController(port: Int, name: String): CommandPS5Controller(port) {
    private fun filterNan(input: Double): Double =
        if (input.isInfinite() || input.isNaN()) 0.0 else input

    private fun rotationEquation(x: Double): Double {
        return 0.2 * x.pow(3) + 0.5 * x
    }

    private val deadband by tunable(DEFAULT_DEADBAND, "$name/deadband")
    private val invertForward by tunable(false, "$name/invertForward")
    private val invertStrafe by tunable(false, "$name/invertStrafe")
    private val invertRotation by tunable(true, "$name/invertRotation")

    private var forward = 0.0
    private var strafe = 0.0
    private var rotation = 0.0
    private var scalar by logged(0.0, "$name/scalar")
    private var chassisPowers by logged(ChassisPowers(), "$name/chassisPowers")

    val swerveOutput: ChassisPowers get() {
        forward = filterNan(if (DRIVER_RIGHT_HANDED) rightY else leftY)
        forward = applyDeadband(forward, deadband)
        if (invertForward) forward *= -1

        strafe = filterNan(if (DRIVER_RIGHT_HANDED) rightX else leftX)
        strafe = applyDeadband(strafe, deadband)
        if (invertStrafe) strafe *= -1

        rotation = filterNan(if (DRIVER_RIGHT_HANDED) leftX else rightX)
        rotation = applyDeadband(rotation, deadband) // squares while keeping the sign
        rotation = rotation.squareMagnitude() * 0.8
        rotation = rotationEquation(rotation) * if (invertRotation) -1 else 1

        scalar = if (DRIVER_RIGHT_HANDED) r2Axis else l2Axis
        scalar = 1 / (2 * scalar + 1)

        chassisPowers.xPower = forward * scalar
        chassisPowers.yPower = strafe * scalar
        chassisPowers.rotationPower = rotation * scalar

        return chassisPowers
    }
}