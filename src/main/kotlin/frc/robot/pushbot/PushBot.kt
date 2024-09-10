package frc.robot.pushbot

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import kcommand.commandbuilder.buildCommand
import kcommand.setDefaultRunCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDriveConstants
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.abs

/**
 * A testbed robot used for driver practice and USAYPT.
 */
class PushBot: ChargerRobot() {
    private val drivetrain = if (isSimulation()){
        EncoderDifferentialDrivetrain.simulated(
            motorType = DCMotor.getNEO(1),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = false)
        )
    }else{
        EncoderDifferentialDrivetrain(
            topLeft = ChargerSparkMax(15).configure(inverted = true),
            topRight = ChargerSparkMax(7),
            bottomLeft = ChargerSparkMax(11).configure(inverted = true),
            bottomRight = ChargerSparkMax(23),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = false)
        )
    }

    private val xboxController = CommandXboxController(0)

    private val shakePower by tunable(0.2)

    init{
        drivetrain.setDefaultRunCommand {
            val precisionModePower = abs(xboxController.leftTriggerAxis).mapBetweenRanges(0.0..1.0, 1.0..6.0)
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

                loopForDuration(0.5){
                    drivetrain.arcadeDrive(shakePower, 0.0, squareInputs = false)
                }

                loopForDuration(0.5){
                    drivetrain.arcadeDrive(-shakePower, 0.0, squareInputs = false)
                }
            }.repeatedly()
        )
    }
}