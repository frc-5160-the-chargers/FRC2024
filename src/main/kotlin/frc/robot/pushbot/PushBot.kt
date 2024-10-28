package frc.robot.pushbot

import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import frc.chargers.framework.*
import kcommand.commandbuilder.buildCommand
import kcommand.setDefaultRunCommand
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDriveConstants
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import monologue.Monologue
import kotlin.math.abs

/**
 * A testbed robot used for driver practice and USAYPT.
 */
class PushBot: ChargerRobot() {
    private val leftMotors = listOf(
        ChargerSparkMax(15, faultLogName = "L1").configure(inverted = false),
        //ChargerSparkMax(11, faultLogName = "L2").configure(inverted = true)
    )

    private val rightMotors = listOf(
        ChargerSparkMax(7, faultLogName = "R1").configure(inverted = false),
        //ChargerSparkMax(23, faultLogName = "R2").configure(inverted = false)
    )

    private val drivetrain = if (isSimulation()){
        EncoderDifferentialDrivetrain.simulated(
            name = "Drivetrain",
            motorType = DCMotor.getNEO(2),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = true)
        )
    }else{
        EncoderDifferentialDrivetrain(
            name = "Drivetrain",
            leftMotors = leftMotors,
            rightMotors = rightMotors,
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = true)
        )
    }

    private val xboxController = CommandXboxController(1)

    private val shakePower by tunable(0.4)

    init{
        Monologue.setupMonologue(this, "", Monologue.MonologueConfig())
        Tunable.tuningMode = true
        drivetrain.setDefaultRunCommand {
            val precisionModePower = abs(xboxController.leftTriggerAxis) * 5.0 + 1.0
            drivetrain.arcadeDrive(
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

                runOnce {
                    log("USAYPT-testing/time", Timer.getFPGATimestamp())
                }

                loopUntil({drivetrain.distanceTraveled > 0.1.meters}){
                    drivetrain.arcadeDrive(shakePower, 0.0, squareInputs = false)
                    println(shakePower)
                }

                loopUntil({drivetrain.distanceTraveled < -0.1.meters}){
                    drivetrain.arcadeDrive(-shakePower, 0.0, squareInputs = false)
                    println(shakePower)
                }
            }.repeatedly()
        )
    }
}