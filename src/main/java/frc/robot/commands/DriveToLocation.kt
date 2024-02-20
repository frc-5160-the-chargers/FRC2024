@file:Suppress("unused")
package frc.robot.commands

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.*
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull

private val DRIVE_TO_LOCATION_AIMING_PID = PIDConstants(0.0115, 0.0,0.004)
private val DRIVE_TO_LOCATION_PRECISION = Precision.Within(Scalar(0.5))
private const val DISTANCE_TO_TAG_REACH_KP = 0.5


enum class FieldLocation(
    val blueAllianceApriltagId: Int,
    val redAllianceApriltagId: Int,
    val pivotAngle: Angle,
    val targetDistanceToTag: Distance? = null
){
    SOURCE_LEFT(
        blueAllianceApriltagId = 2,
        redAllianceApriltagId = 10,
        pivotAngle = PivotAngle.SOURCE,
        0.5.meters
    ), // tbd

    SOURCE_RIGHT(
        blueAllianceApriltagId = 1,
        redAllianceApriltagId = 9,
        pivotAngle = PivotAngle.SOURCE,
        0.5.meters
    ), // tbd

    AMP(
        blueAllianceApriltagId = 6,
        redAllianceApriltagId = 5,
        pivotAngle = PivotAngle.AMP,
        0.5.meters
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
): Command = buildCommand( logIndividualCommands = true) {
    val targetId = when (DriverStation.getAlliance().getOrNull()){
        DriverStation.Alliance.Blue, null -> target.blueAllianceApriltagId

        DriverStation.Alliance.Red -> target.redAllianceApriltagId
    }

    // represents raw, unflipped pose fetched from apriltagfieldlayout
    val targetPoseRaw = ChargerRobot
        .APRILTAG_LAYOUT
        .getTagPose(targetId)
        .orElseThrow()

    val targetPose: UnitPose2d =
        targetPoseRaw
            .toPose2d()
            .ofUnit(meters)
            .flipWhenNeeded()

    val targetHeight: Distance =
        targetPoseRaw.z.ofUnit(meters)

    val aimingController by getOnceDuringRun {
        SuperPIDController(
            DRIVE_TO_LOCATION_AIMING_PID,
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

        return (aimingError in DRIVE_TO_LOCATION_PRECISION.allowableError).also{
            Logger.recordOutput("AimToLocation/hasFinishedAiming", it)
        }
    }

    fun getDistanceErrorToTag(): Distance {
        if (target.targetDistanceToTag == null) return 0.meters

        val distance = apriltagVision.robotToTargetDistance(targetHeight) ?: return 0.meters

        return (distance - target.targetDistanceToTag).also{
            Logger.recordOutput("AimToLocation/distanceTest", it.siValue)
        }
    }

    addRequirements(drivetrain, pivot)

    runOnce{
        apriltagVision.reset()
    }

    if (path != null){
        val distanceFromAprilTag by getOnceDuringRun {
            val drivetrainPose = drivetrain.poseEstimator.robotPose
            val distanceX = drivetrainPose.x - targetPose.x
            val distanceY = drivetrainPose.y - targetPose.y
            hypot(distanceX, distanceY)
        }

        val distanceFromPathStart by getOnceDuringRun {
            val drivetrainPose = drivetrain.poseEstimator.robotPose
            (drivetrainPose.translation - path.previewStartingHolonomicPose.translation.ofUnit(meters)).norm
        }

        runIf(
            {distanceFromAprilTag > 1.3.meters && distanceFromPathStart > 0.3.meters},
            onTrue = AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS),
            onFalse = runIf(
                {distanceFromAprilTag > 0.6.meters},
                AutoBuilder.followPath(path)
            )
        )
    }

    runParallelUntilAllFinish {
        +pivot.setAngleCommand(target.pivotAngle)

        loopUntil({ (!canFindTarget() || hasFinishedAiming()) && getDistanceErrorToTag() <= Distance(0.003) }){
            Logger.recordOutput("AimToLocation/isAiming", true)
            drivetrain.swerveDrive(
                xPower = getDistanceErrorToTag().siValue * DISTANCE_TO_TAG_REACH_KP,
                yPower = -aimingController.calculateOutput().siValue,
                rotationPower = 0.0,
                fieldRelative = false
            )
        }
    }

    onEnd{
        Logger.recordOutput("AimToLocation/isAiming", false)
        drivetrain.setDriveVoltages(
            listOf(0.volts, 0.volts, 0.volts, 0.volts)
        )
    }
}