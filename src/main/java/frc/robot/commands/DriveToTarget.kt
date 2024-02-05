@file:Suppress("unused")
package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import java.util.*




enum class DriveToTargetLocation(
    // no need for red alliance pose; flip is manually applied
    val path: PathPlannerPath,
    val blueAllianceApriltagId: Int,
    val redAllianceApriltagId: Int,
    val shooterAngle: PivotAngle,
){
    SOURCE_LEFT(
        path = PathPlannerPath.fromPathFile("SourceLeftTeleop"),
        blueAllianceApriltagId = 2,
        redAllianceApriltagId = 10,
        shooterAngle = PivotAngle.SOURCE
    ), // tbd

    SOURCE_RIGHT(
        path = PathPlannerPath.fromPathFile("SourceRightTeleop"),
        blueAllianceApriltagId = 1,
        redAllianceApriltagId = 9,
        shooterAngle = PivotAngle.SOURCE
    ), // tbd

    AMP(
        path = PathPlannerPath.fromPathFile("AmpTeleop"),
        blueAllianceApriltagId = 6,
        redAllianceApriltagId = 5,
        shooterAngle = PivotAngle.AMP
    )
}

fun driveToTarget(
    target: DriveToTargetLocation,
    aimingPID: PIDConstants = OPEN_LOOP_TRANSLATION_PID,
    pathfind: Boolean = true,

    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: AprilTagVisionPipeline,
    shooter: Shooter,
): Command = buildCommand {

    runOnce{
        visionIO.requireAndReset()
    }

    runParallelUntilAllFinish {
        +shooter.setAngleCommand(target.shooterAngle)

        runSequentially{
            if (pathfind){
                AutoBuilder.pathfindThenFollowPath(target.path, PATHFIND_CONSTRAINTS)
            }

            +aimToAprilTag(
                if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)) {
                    target.redAllianceApriltagId
                } else {
                    target.blueAllianceApriltagId
                },
                aimingPID,
                drivetrain = drivetrain,
                visionIO = visionIO
            )
        }
    }

    runOnce{
        visionIO.removeRequirement()
    }
}