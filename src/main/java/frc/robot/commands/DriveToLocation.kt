@file:Suppress("unused")
package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.CAMERA_YAW_TO_OPEN_LOOP_STRAFE_PID
import frc.robot.NO_TARGET_FOUND_ALERT
import frc.robot.OPEN_LOOP_STRAFE_PRECISION
import frc.robot.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull

enum class FieldLocation(
    val blueAllianceApriltagId: Int,
    val redAllianceApriltagId: Int,
    val pivotAngle: Angle,
){
    SOURCE_LEFT(
        blueAllianceApriltagId = 2,
        redAllianceApriltagId = 10,
        pivotAngle = PivotAngle.SOURCE
    ), // tbd

    SOURCE_RIGHT(
        blueAllianceApriltagId = 1,
        redAllianceApriltagId = 9,
        pivotAngle = PivotAngle.SOURCE
    ), // tbd

    AMP(
        blueAllianceApriltagId = 6,
        redAllianceApriltagId = 5,
        pivotAngle = PivotAngle.AMP
    )
}

/**
 * Aims to a location on the field, and paths to there if applicable.
 */
fun driveToLocation(
    target: FieldLocation,
    path: PathPlannerPath? = null,

    drivetrain: EncoderHolonomicDrivetrain,
    apriltagVision: AprilTagVisionPipeline,
    pivot: Pivot,
): Command = buildCommand ( logIndividualCommands = true) {
    val targetId = when (DriverStation.getAlliance().getOrNull()){
        DriverStation.Alliance.Blue, null -> target.blueAllianceApriltagId

        DriverStation.Alliance.Red -> target.redAllianceApriltagId
    }
    val targetPose: UnitPose2d =
        ChargerRobot
            .APRILTAG_LAYOUT
            .getTagPose(targetId)
            .orElseThrow()
            .toPose2d()
            .ofUnit(meters)
            .flipWhenNeeded()

    val aimingController by getOnceDuringRun {
        SuperPIDController(
            CAMERA_YAW_TO_OPEN_LOOP_STRAFE_PID,
            getInput = { Scalar(apriltagVision.bestTarget?.tx ?: 0.0) },
            target = Scalar(0.0),
            outputRange = Scalar(-0.5)..Scalar(0.5)
        )
    }

    fun canFindTarget(): Boolean {
        val bestTarget = apriltagVision.bestTarget

        return if (bestTarget == null){
            println("NO TARGET FOUND!")
            NO_TARGET_FOUND_ALERT.active = true
            false
        }else if (targetId != bestTarget.fiducialId){
            val alertText = "An apriltag command can find an apriltag, " +
                    "but it does not match the id of $targetId. " +
                    "(Found id: " + bestTarget.fiducialId

            Alert.warning(text = alertText).active = true
            println(alertText)
            false
        }else{
            true
        }.also{
            Logger.recordOutput("AimToLocation/canFindTarget", it)
        }
    }

    fun hasFinishedAiming(): Boolean {
        val aimingError = aimingController.error
        Logger.recordOutput("AimToLocation/aimingError", aimingError.siValue)
        return (aimingError in OPEN_LOOP_STRAFE_PRECISION.allowableError).also{
            Logger.recordOutput("AimToLocation/hasFinishedAiming", it)
        }
    }


    var aimLoopOccurences by resetDuringRun{0}

    addRequirements(drivetrain, pivot)

    runOnce{
        apriltagVision.reset()
        println(targetPose)
    }

    runParallelUntilAllFinish {
        +pivot.setAngleCommand(target.pivotAngle)

        runSequentially{
            if (path != null){
                runIf(
                    { (abs(drivetrain.poseEstimator.robotPose.translation.norm - targetPose.translation.norm) > 1.meters).also{ println("Pathfinding?$it") } },
                    onTrue = AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS),
                    onFalse = AutoBuilder.followPath(path)
                )
            }

            loopUntil({ !canFindTarget() || hasFinishedAiming() }){
                Logger.recordOutput("AimToLocation/isAiming", true)
                Logger.recordOutput("AimToLocation/aimLoop#", aimLoopOccurences)
                aimLoopOccurences++
                drivetrain.swerveDrive(
                    xPower = 0.0,
                    yPower = -aimingController.calculateOutput().siValue,
                    rotationPower = 0.0,
                    fieldRelative = false
                )
            }

            runOnce{
                Logger.recordOutput("AimToLocation/isAiming", false)
                drivetrain.setDriveVoltages(
                    listOf(0.volts, 0.volts, 0.volts, 0.volts)
                )
                drivetrain.stopInX()
            }
        }
    }
}