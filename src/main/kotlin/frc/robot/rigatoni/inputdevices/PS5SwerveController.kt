package frc.robot.rigatoni.inputdevices

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.chargers.framework.logged
import frc.chargers.framework.tunable
import frc.chargers.utils.squareMagnitude
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.rigatoni.DEFAULT_DEADBAND
import frc.robot.rigatoni.DRIVER_RIGHT_HANDED
import kotlin.math.abs
import kotlin.math.pow

class PS5SwerveController(port: Int): CommandPS5Controller(port) {
    private fun filterNan(input: Double): Double =
        if (input.isInfinite() || input.isNaN()) 0.0 else input

    private fun rotationEquation(x: Double): Double {
        return 0.2 * x.pow(3) + 0.5 * x
    }

    private val deadband by tunable(DEFAULT_DEADBAND, "DriverController/deadband")
    private val invertForward by tunable(RobotBase.isReal(), "DriverController/invertForward")
    private val invertStrafe by tunable(RobotBase.isReal(), "DriverController/invertStrafe")
    private val invertRotation by tunable(RobotBase.isReal(), "DriverController/invertRotation")

    private var forward = 0.0
    private var strafe = 0.0
    private var rotation = 0.0
    private var scalar by logged(0.0, "DriverController/scalar")
    private var chassisPowers by logged(ChassisPowers(), "DriverController/chassisPowers")

    val swerveOutput: ChassisPowers get() {
        forward = filterNan(if (DRIVER_RIGHT_HANDED) rightY else leftY)
        forward = applyDeadband(forward, deadband)
        if (invertForward) forward *= -1

        strafe = filterNan(if (DRIVER_RIGHT_HANDED) rightX else leftX)
        strafe = applyDeadband(strafe, deadband)
        if (invertStrafe) strafe *= -1

        rotation = filterNan(if (DRIVER_RIGHT_HANDED) leftX else rightX)
        rotation = applyDeadband(rotation, deadband).squareMagnitude() // squares while keeping the sign
        rotation = rotationEquation(rotation) * if (invertRotation) -1 else 1

        scalar = abs(filterNan(if (DRIVER_RIGHT_HANDED) r2Axis else l2Axis))
        scalar = 1 / (6 * abs(scalar) + 1)

        chassisPowers.xPower = forward * scalar
        chassisPowers.yPower = strafe * scalar
        chassisPowers.rotationPower = rotation * scalar

        return chassisPowers
    }
}