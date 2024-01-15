@file:Suppress("unused")

package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit

fun basicTaxi(drivetrain: EncoderHolonomicDrivetrain, power: Double = -0.2): Command =
    buildCommand("Taxi Auto"){
        addRequirements(drivetrain)

        loopFor(3.seconds){
            drivetrain.swerveDrive(power,0.0,0.0)
        }

        runOnce{
            drivetrain.stop()
        }
    }

fun pathplannerTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    resetPoseAtStart: Boolean = true
): Command =
    buildCommand("Taxi Auto w/ Path") {
        addRequirements(drivetrain)

        val path = PathPlannerPath.fromPathFile("Taxi Path")

        runOnce{
            if (resetPoseAtStart){
                drivetrain.poseEstimator.resetPose(
                    path.previewStartingHolonomicPose.ofUnit(meters)
                )
                println("Pose reset for drivetrain.")
            }
        }

        +AutoBuilder.followPath(path)
    }