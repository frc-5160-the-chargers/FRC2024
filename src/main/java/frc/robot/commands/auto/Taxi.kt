@file:Suppress("unused")

package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun basicTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    power: Double = -0.3,

    // optional subsystem parameters to zero before taxi
    shooter: Shooter? = null,
    groundIntake: GroundIntake? = null,
): Command =
    buildCommand("Taxi Auto"){
        addRequirements(drivetrain)

        loopFor(3.seconds){
            drivetrain.swerveDrive(power,0.0,0.0)
        }

        // stops subsystems
        runOnce{
            shooter?.setSpeed(0.0)
            shooter?.setPivotSpeed(0.0)
            groundIntake?.spin(0.0)
        }

        runParallelUntilAllFinish{
            if (shooter != null){
                +shooter.setAngleCommand(PivotAngle.IDLE)
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
            shooter?.setSpeed(0.0)
            shooter?.setPivotSpeed(0.0)
            groundIntake?.spin(0.0)
        }

        runParallelUntilAllFinish{
            if (shooter != null){
                +shooter.setAngleCommand(PivotAngle.IDLE)
            }

            // follows path
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
        }
    }