@file:Suppress("unused")

package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun basicTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    power: Double = -0.3,

    // optional subsystem parameters to zero before taxi
    shooter: Shooter? = null,
    pivot: Pivot? = null,
    groundIntake: GroundIntake? = null,
): Command =
    buildCommand("Taxi Auto"){
        addRequirements(drivetrain)

        loopFor(3.seconds){
            drivetrain.swerveDrive(power,0.0,0.0)
        }

        // stops subsystems
        runOnce{
            shooter?.intake(0.0)
            pivot?.setVoltage(0.volts)
            groundIntake?.intake(0.0)
        }

        runParallelUntilAllFinish{
            if (pivot != null){
                +pivot.setAngleCommand(PivotAngle.STOWED)
            }

            // primary drive function
            loopFor(3.seconds){
                drivetrain.swerveDrive(power,0.0,0.0, fieldRelative = false)
            }
        }

        runOnce{
            drivetrain.stop()
        }
    }

fun pathplannerTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    pathName: String = "Taxi Path",
    resetPoseAtStart: Boolean = true,

    // optional subsystem parameters to zero before taxi
    shooter: Shooter? = null,
    pivot: Pivot? = null,
    groundIntake: GroundIntake? = null,
): Command =
    buildCommand("Taxi Auto w/ Path") {
        addRequirements(drivetrain)
        if (shooter != null) addRequirements(shooter)
        if (groundIntake != null) addRequirements(groundIntake)

        runOnce{
            if (resetPoseAtStart){
                drivetrain.poseEstimator.resetToPathplannerTrajectory(pathName)
                println("Pose reset for drivetrain.")
            }
        }

        // stops subsystems
        runOnce{
            shooter?.setIdle()
            groundIntake?.setIdle()
        }

        runParallelUntilAllFinish{
            if (pivot != null){
                +pivot.setAngleCommand(PivotAngle.STOWED)
            }

            // follows path
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
        }
    }