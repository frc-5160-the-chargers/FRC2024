@file:Suppress("unused")
package frc.robot.commands.apriltag

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.commands.pathfindOptimal
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import java.util.*

enum class DriveToTargetGoal(
    // no need for red alliance pose; flip is manually applied
    val blueAlliancePose: UnitPose2d,
    val blueAllianceApriltagId: Int,
    val redAllianceApriltagId: Int,
    val shooterAngle: PivotAngle,
){
    SOURCE_LEFT(
        blueAlliancePose = UnitPose2d(),
        blueAllianceApriltagId = 2,
        redAllianceApriltagId = 10,
        shooterAngle = PivotAngle.SOURCE
    ), // tbd

    SOURCE_RIGHT(
        blueAlliancePose = UnitPose2d(),
        blueAllianceApriltagId = 1,
        redAllianceApriltagId = 9,
        shooterAngle = PivotAngle.SOURCE
    ), // tbd

    AMP(
        UnitPose2d(1.85.meters, 7.6.meters, 0.degrees),
        blueAllianceApriltagId = 6,
        redAllianceApriltagId = 5,
        shooterAngle = PivotAngle.AMP
    )
}

fun driveToTarget(
    target: DriveToTargetGoal,
    aimingPID: PIDConstants = DEFAULT_AIMING_PID,
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: AprilTagVisionPipeline,
    shooter: Shooter? = null,
    pathfind: Boolean = true
): Command = buildCommand {
    runParallelUntilAllFinish {
        if (shooter != null){
            runSequentially{
                runOnce(shooter){ shooter.setPivotPosition(target.shooterAngle) }

                loopUntil({ shooter.hasHitPivotTarget }, shooter){
                    shooter.setPivotPosition(target.shooterAngle)
                }

                runOnce(shooter){ shooter.setPivotVoltage(0.volts) }
            }
        }

        runSequentially{
            if (pathfind){
                +drivetrain.pathfindOptimal(target.blueAlliancePose)
            }

            +aimToApriltag(
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
}