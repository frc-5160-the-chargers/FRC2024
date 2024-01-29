package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.equations.epsilonEquals
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.constants.DRIVER_CONTROLLER_PORT
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.pow


object DriverController: CommandXboxController(DRIVER_CONTROLLER_PORT){

    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.1


    /* Public API */
    val pointNorthButton: Trigger = y()
    val pointSouthButton: Trigger = a()
    val pointEastButton: Trigger = x()
    val pointWestButton: Trigger = b()


    /* Private implementation */

    private fun InputAxis.applyDefaults(): InputAxis =
        this.applyDeadband(DEFAULT_DEADBAND)


    private val driveScalar =
        InputAxis{ leftX.pow(2) + leftY.pow(2) }
            .square()
            .applyMultiplier(0.8)

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
            //.applyMultiplier(if (isReal()) -1.0 else 1.0)
            .withModifier{ it * getScaleRate() }

    private val strafeAxis =
        InputAxis{ leftX }
            //.applyMultiplier(if (isReal()) -1.0 else 1.0)
            .withModifier{ it * getScaleRate() }

    private val rotationAxis =
        InputAxis{ rightX }
            .applyDefaults()
            .square()
            .applyEquation( Polynomial(0.1,0.0,0.4,0.0) )


    private val turboAxis =
        InputAxis{ leftTriggerAxis }
            .applyDefaults()
            .mapToRange(1.0..2.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }

    private val precisionAxis =
        InputAxis{ leftTriggerAxis }
            .applyDefaults()
            .mapToRange(1.0..4.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .withModifier{ 1.0 / it }



    val swerveOutput: ChassisPowers get(){
        val forward = forwardAxis()
        val strafe = strafeAxis()
        val rotation = rotationAxis()

        val turbo = turboAxis()
        val precision = precisionAxis()

        recordOutput("Drivetrain(Swerve)/xPower", forward * turbo * precision)
        recordOutput("Drivetrain(Swerve)/yPower", strafe * turbo * precision)
        recordOutput("Drivetrain(Swerve)/rotation output", rotation * precision)

        return ChassisPowers(
            xPower = forward * turbo * precision,
            yPower = strafe * turbo * precision,
            rotationPower = rotation * precision
        )
    }
}