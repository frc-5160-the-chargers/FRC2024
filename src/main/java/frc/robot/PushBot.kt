package frc.robot

import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDriveConstants
import frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.wpilibextensions.kinematics.ChassisPowers

@Suppress("unused")
class PushBot: ChargerRobot() {
    private val drivetrain = EncoderDifferentialDrivetrain(
        topLeft = ChargerSparkMax(15),
        topRight = ChargerSparkMax(7),
        bottomLeft = ChargerSparkMax(11),
        bottomRight = ChargerSparkMax(23),
        constants = DifferentialDriveConstants.andymarkKitbot(invertMotors = true)
    ){
        smartCurrentLimit = SmartCurrentLimit(40.amps)
        voltageCompensationNominalVoltage = 12.volts
        openLoopRampRate = 48.0
        closedLoopRampRate = 48.0
    }

    private val xboxController = CommandXboxController(1)

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
    }
}