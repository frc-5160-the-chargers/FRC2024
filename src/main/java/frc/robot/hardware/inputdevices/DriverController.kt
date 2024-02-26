@file:Suppress("unused")
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

    enum class Driver {
        NAYAN, KENNA, CONRAD
    }


    /* Top-Level constants */
    private const val DEFAULT_DEADBAND = 0.2
    private const val IS_SIM_XBOX_CONTROLLER = false
    private val DRIVER = Driver.NAYAN


    /* Public API */
    val pointNorthTrigger: Trigger = y()
    val pointSouthTrigger: Trigger = a()
    val pointEastTrigger: Trigger = x()
    val pointWestTrigger: Trigger = b()

    val disableFieldRelativeTrigger: Trigger = start().or(back())


    /* Private implementation */

    private fun InputAxis.applyDefaults(): InputAxis =
        this.applyDeadband(DEFAULT_DEADBAND)


    private val driveScalar =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> sqrt(leftX.pow(2) + leftY.pow(2))

                Driver.KENNA, Driver.CONRAD -> sqrt(rightX.pow(2) + rightY.pow(2))
            }
        }
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
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> leftY

                Driver.KENNA, Driver.CONRAD -> rightY
            }
        }
            .applyDeadband(DEFAULT_DEADBAND)
            .withModifier{ if (IS_SIM_XBOX_CONTROLLER) -1.0 * it else it }
            .withModifier{ it * getScaleRate() }

    private val strafeAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> leftX

                Driver.KENNA, Driver.CONRAD -> rightX
            }
        }
            .applyDeadband(0.3)
            .withModifier{ if (IS_SIM_XBOX_CONTROLLER) -1.0 * it else it }
            .withModifier{ it * getScaleRate() }

    private val rotationAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> rightX

                Driver.KENNA, Driver.CONRAD -> leftX
            }
        }
            .applyDeadband(DEFAULT_DEADBAND)
            .square()
            .applyEquation( Polynomial(-0.1,0.0,-0.4,0.0) )


    private val turboAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> leftTriggerAxis

                Driver.KENNA, Driver.CONRAD -> rightTriggerAxis
            }
        }
            .mapToRange(1.0..2.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }

    private val precisionAxis =
        InputAxis{
            when (DRIVER){
                Driver.NAYAN -> rightTriggerAxis

                Driver.KENNA, Driver.CONRAD -> leftTriggerAxis
            }
        }
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