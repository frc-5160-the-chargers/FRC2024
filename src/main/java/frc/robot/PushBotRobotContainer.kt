@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.units.amps
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.subsystems.differentialdrive.sparkMaxDrivetrain
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.hardware.inputdevices.DriverController

class PushBotRobotContainer: ChargerRobotContainer() {
    private val drivetrain = sparkMaxDrivetrain(
        topLeft = ChargerSparkMax(15){ inverted = true },
        topRight = ChargerSparkMax(7),
        bottomLeft = ChargerSparkMax(11){ inverted = true },
        bottomRight = ChargerSparkMax(23)
    ){
        smartCurrentLimit = SmartCurrentLimit(40.amps)
    }

    init{
        drivetrain.setDefaultRunCommand {
            val swerveOutput = DriverController.swerveOutput
            drivetrain.curvatureDrive(
                ChassisPowers(swerveOutput.xPower, 0.0, -swerveOutput.rotationPower)
            )
        }
    }


    override val autonomousCommand: Command
        get() = Commands.none()
}