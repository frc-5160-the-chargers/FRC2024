package frc.robot.pushbot

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import dev.doglog.DogLogOptions
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.InstantCommand
import kcommand.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.framework.*
import kcommand.commandbuilder.buildCommand
import kcommand.setDefaultRunCommand
import frc.chargers.framework.HorseLog.log
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDriveConstants
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.utils.mapControllerInput
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import kotlin.math.abs

/**
 * A testbed robot used for driver practice and USAYPT.
 */
class PushBot: ChargerRobot() {
    private val leftMotors = listOf(
        ChargerSparkMax(15, faultLogName = "L1").configure(inverted = false),
        //ChargerSparkMax(11, faultLogName = "L2").configure(inverted = false)
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
    private val shakeDistance by tunable(0.1.meters)

    private var cond = false

    init{
        SmartDashboard.putData(
            "ZeroEncoders",
            InstantCommand {
                (leftMotors + rightMotors).forEach{ it.configure(currentPosition = 0.degrees) }
            }
        )

        Trigger{ cond }.onTrue(InstantCommand({println("hi!!!!")}))

        HorseLog.setOptions(DogLogOptions().withNtPublish(true))
        Tunable.tuningMode = true
        drivetrain.setDefaultRunCommand {
            val precisionModePower = abs(xboxController.leftTriggerAxis).mapControllerInput(1.0..6.0)
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
                    log("USAYPT-testing/time", fpgaTimestamp())
                }

                loopUntil({drivetrain.distanceTraveled > shakeDistance}){
                    drivetrain.arcadeDrive(shakePower, 0.0, squareInputs = false)
                }

                loopUntil({drivetrain.distanceTraveled < -shakeDistance}){
                    drivetrain.arcadeDrive(-shakePower, 0.0, squareInputs = false)
                    println(shakePower)
                }
            }.repeatedly()
        )
    }

    override fun autonomousInit() {
        cond = true
    }
}