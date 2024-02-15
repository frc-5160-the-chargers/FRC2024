@file:Suppress("unused")
package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.constants.PID
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter



fun grabGamepiece(
    path: PathPlannerPath? = null,
    noteDetector: ObjectVisionPipeline,

    drivetrain: EncoderHolonomicDrivetrain,
    pivot: Pivot,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake, pivot)

    runOnce{
        noteDetector.reset()

        drivetrain.setRotationOverride(
            AimToObjectRotationOverride(
                noteDetector,
                PID.CAMERA_YAW_TO_ROTATIONAL_VELOCITY
            )
        )
    }

    runParallelUntilFirstCommandFinishes{
        // parallel #1
        runSequentially{
            if (path != null){
                +AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS)
            }

            // fieldRelative = false because rotation override makes drivetrain aim to gamepiece;
            // this means that driving back while field relative is not true will directly grab the gamepiece
            loopWhile( { shooter.hasBeamBreakSensor && !shooter.hasGamepiece } ){
                drivetrain.swerveDrive(-0.15,0.0,0.0, fieldRelative = false)
            }
        }

        // parallel #2
        +runGroundIntake(
            shooter, pivot, groundIntake
        )
    }

    runOnce{
        drivetrain.stop()
        drivetrain.removeRotationOverride()
        shooter.setIdle()
        groundIntake.setIdle()
    }
}