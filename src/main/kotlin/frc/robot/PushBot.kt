package frc.robot

import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDriveConstants
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.wpilibextensions.kinematics.ChassisPowers

/**
 * A testbed robot used for driver practice and USAYPT.
 */
class PushBot: ChargerRobot() {
    private val drivetrain = if (isSimulation()){
        EncoderDifferentialDrivetrain(
            topLeft = MotorSim(DCMotor.getNEO(1)),
            topRight = MotorSim(DCMotor.getNEO(1)),
            bottomLeft = MotorSim(DCMotor.getNEO(1)),
            bottomRight = MotorSim(DCMotor.getNEO(1)),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = false)
        )
    }else{
        EncoderDifferentialDrivetrain(
            topLeft = ChargerSparkMax(15){ inverted = true },
            topRight = ChargerSparkMax(7),
            bottomLeft = ChargerSparkMax(11){ inverted = true },
            bottomRight = ChargerSparkMax(23),
            constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = false)
        ){
            smartCurrentLimit = SmartCurrentLimit(40.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        }
    }

    private val xboxController = CommandXboxController(0)

    private val shakePower by tunable(0.2)

    init{
        drivetrain.setDefaultRunCommand {
            val precisionModePower = xboxController.leftTriggerAxis.mapBetweenRanges(0.0..1.0, 1.0..6.0)
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
                addRequirements(drivetrain)

                loopFor(0.5.seconds){
                    drivetrain.arcadeDrive(shakePower, 0.0, squareInputs = false)
                }

                loopFor(0.5.seconds){
                    drivetrain.arcadeDrive(-shakePower, 0.0, squareInputs = false)
                }
            }.repeatedly()
        )
    }
}