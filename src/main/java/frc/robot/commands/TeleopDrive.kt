package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.hardware.inputdevices.DriverController

fun teleopDrive(
    drivetrain: EncoderHolonomicDrivetrain
): Command = buildCommand{
    addRequirements(drivetrain)

    /*
    if (RobotBase.isSimulation()){
        runOnce{
            drivetrain.poseEstimator.zeroPose()
        }
    }
     */

    var swerveOutput: ChassisPowers

    loopForever{
        swerveOutput = DriverController.swerveOutput(drivetrain.heading)
        drivetrain.swerveDrive(swerveOutput)
    }

    onEnd{
        drivetrain.stop()
    }
}