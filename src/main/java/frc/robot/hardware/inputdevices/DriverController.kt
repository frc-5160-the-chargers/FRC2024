package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.equations.epsilonEquals
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.DRIVER_CONTROLLER_PORT
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.pow
import kotlin.math.sqrt


object DriverController: CommandXboxController(DRIVER_CONTROLLER_PORT){
    var isSimXboxController = false


    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.2


    /* Public API */
    val pointNorthButton: Trigger = y()
    val pointSouthButton: Trigger = a()
    val pointEastButton: Trigger = x()
    val pointWestButton: Trigger = b()


    /* Private implementation */

    private fun InputAxis.applyDefaults(): InputAxis =
        this.applyDeadband(DEFAULT_DEADBAND)


    private val driveScalar =
        InputAxis{ sqrt(leftX.pow(2) + leftY.pow(2)) }
            .applyMultiplier(0.6)

    private fun getScaleRate(): Double{
        val baseValue = driveScalar.getBaseValue()
        val scaledValue = driveScalar()

        return if (baseValue epsilonEquals 0.0){
            1.0
        }else {
            scaledValue / baseValue
        }.also{ recordOutput("Drivetrain(Swerve)/scaleRate", it) }
    }




    private val forwardAxis =
        InputAxis{ leftY }
            .applyDefaults()
            .withModifier{ if (isSimXboxController) -1.0 * it else it }
            .withModifier{ it * getScaleRate() }

    private val strafeAxis =
        InputAxis{ leftX }
            .applyDeadband(0.3)
            .withModifier{ if (isSimXboxController) -1.0 * it else it }
            .withModifier{ it * getScaleRate() }

    private val rotationAxis =
        InputAxis{ rightX }
            .applyDefaults()
            .square()
            .applyEquation( Polynomial(-0.1,0.0,-0.4,0.0) )


    private val turboAxis =
        InputAxis{ rightTriggerAxis }
            .mapToRange(1.0..2.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }

    private val precisionAxis =
        InputAxis{ leftTriggerAxis }
            .mapToRange(1.0..4.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .withModifier{ 1.0 / it }



    val swerveOutput: ChassisPowers get(){
        val forward = forwardAxis()
        val strafe = strafeAxis()
        val rotation = rotationAxis()

        val turbo = turboAxis()
        val precision = precisionAxis()

        recordOutput("DriverController/TurboOutput", turbo)
        recordOutput("DriverController/PrecisionOutput", precision)


        recordOutput("DriverController/xPower", forward)
        recordOutput("DriverController/yPower", strafe)
        recordOutput("DriverController/rotation", rotation)

        return ChassisPowers(
            xPower = forward * turbo * precision,
            yPower = strafe * turbo * precision,
            rotationPower = rotation * precision
        )
    }
}