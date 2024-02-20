@file:Suppress("unused")

package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain

fun basicTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    power: Double = -0.3,
): Command =
    buildCommand("Taxi Auto"){
        loopFor(3.seconds){
            drivetrain.swerveDrive(power,0.0,0.0)
        }

        runOnce{
            drivetrain.stop()
        }
    }

fun pathplannerTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    pathName: String = "Taxi Path",
    resetPoseAtStart: Boolean = true,
): Command =
    buildCommand("Taxi Auto w/ Path") {
        addRequirements(drivetrain)

        runOnce{
            if (resetPoseAtStart){
                drivetrain.poseEstimator.resetToPathplannerTrajectory(pathName)
                println("Pose reset for drivetrain.")
            }
        }

        +AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
    }