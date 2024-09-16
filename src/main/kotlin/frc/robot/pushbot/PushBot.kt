package frc.robot.pushbot

import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import kcommand.commandbuilder.buildCommand
import kcommand.setDefaultRunCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.tunable
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDriveConstants
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.utils.mapControllerInput
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.abs

/**
 * A testbed robot used for driver practice and USAYPT.
 */
class PushBot: ChargerRobot() {
    private val drivetrain = if (isSimulation()){
        EncoderDifferentialDrivetrain.simulated(
            motorType = DCMotor.getNEO(2),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = false)
        )
    }else{
        EncoderDifferentialDrivetrain(
            leftMotors = listOf(
                ChargerSparkMax(15).configure(inverted = true),
                ChargerSparkMax(11).configure(inverted = true)
            ),
            rightMotors = listOf(
                ChargerSparkMax(7),
                ChargerSparkMax(23)
            ),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = false)
        )
    }

    private val xboxController = CommandXboxController(0)

    private val shakePower by tunable(0.2)

    init{
        drivetrain.setDefaultRunCommand {
            val precisionModePower = abs(xboxController.leftTriggerAxis).mapControllerInput(1.0..6.0)
            drivetrain.curvatureDrive(
                ChassisPowers(
                    MathUtil.applyDeadband(xboxController.leftY, .2) * precisionModePower,
                    0.0,
                    -MathUtil.applyDeadband(xboxController.rightX, .2) * precisionModePower
                )
            )
        }

        autonomous().whileTrue(
            buildCommand {
                require(drivetrain)

                loopUntil({drivetrain.distanceTraveled > 0.1.meters}){
                    drivetrain.arcadeDrive(shakePower, 0.0, squareInputs = false)
                }

                loopUntil({drivetrain.distanceTraveled < -0.1.meters}){
                    drivetrain.arcadeDrive(-shakePower, 0.0, squareInputs = false)
                }
            }.repeatedly()
        )
    }
}