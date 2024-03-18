@file:Suppress("unused")
package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.subsystems.differentialdrive.sparkMaxDrivetrain
import frc.robot.hardware.inputdevices.DriverController

class PushBotRobotContainer: ChargerRobotContainer() {
    private val drivetrain = sparkMaxDrivetrain(
        topLeft = ChargerSparkMax(15){ inverted = true },
        topRight = ChargerSparkMax(7),
        bottomLeft = ChargerSparkMax(11){ inverted = true },
        bottomRight = ChargerSparkMax(23)
    )

    private val gyroIO = ChargerNavX(useFusedHeading = false)

    init{
        drivetrain.setDefaultRunCommand {
            drivetrain.curvatureDrive(DriverController.swerveOutput)
        }
    }


    override val autonomousCommand: Command
        get() = Commands.none()
}