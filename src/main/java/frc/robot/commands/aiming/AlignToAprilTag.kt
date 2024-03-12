@file:Suppress("unused")
package frc.robot.commands.aiming

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.robot.*
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull

private val APRILTAG_AIM_PID = PIDConstants(0.01, 0.0,0.004)
private val APRILTAG_AIM_PRECISION = Precision.Within(Scalar(0.5))
private const val DISTANCE_REACH_KP = 0.6


enum class AprilTagLocation(
    val blueAllianceAprilTagId: Int,
    val redAllianceAprilTagId: Int,
    val pivotAngle: Angle,
    val targetDistanceToTag: Distance? = null
){
    SOURCE_LEFT(
        blueAllianceAprilTagId = 2,
        redAllianceAprilTagId = 9,
        pivotAngle = PivotAngle.SOURCE,
        0.5.meters
    ), // tbd

    SOURCE_RIGHT(
        blueAllianceAprilTagId = 1,
        redAllianceAprilTagId = 10,
        pivotAngle = PivotAngle.SOURCE,
        0.5.meters
    ), // tbd

    AMP(
        blueAllianceAprilTagId = 6,
        redAllianceAprilTagId = 5,
        pivotAngle = PivotAngle.AMP,
        0.5.meters
    ),

    STAGE_LEFT(
        blueAllianceAprilTagId = 0,
        redAllianceAprilTagId = 0,
        pivotAngle = PivotAngle.STOWED,
        0.7.meters
    )
}

/**
 * Aims and drives to an apriltag on a certain location within the field,
 * and moves the pivot to the correct position.
 *
 * A follow path command can optionally be specified,
 * which will be automatically cancelled once the range to the apriltag is small enough.
 * This is to allow for a smoother and quicker line-up with the apriltag target.
 */
fun alignToAprilTag(
    drivetrain: EncoderHolonomicDrivetrain,
    apriltagVision: AprilTagVisionPipeline,
    pivot: Pivot,

    tagLocation: AprilTagLocation,
    followPathCommand: Command? = null
): Command = buildCommand(name = "Aim To AprilTag($tagLocation)", logIndividualCommands = true) {

    // these values have their internal value evaluated once they are accessed.
    val targetId by lazy {
        when (DriverStation.getAlliance().getOrNull()){
            DriverStation.Alliance.Blue, null -> tagLocation.blueAllianceAprilTagId

            DriverStation.Alliance.Red -> tagLocation.redAllianceAprilTagId
        }
    }

    val tagPose: UnitPose3d by lazy {
        ChargerRobot
            .APRILTAG_LAYOUT
            .getTagPose(targetId)
            .orElseThrow()
            .ofUnit(meters)
    }

    val headingToFaceAprilTag by lazy { tagPose.toPose2d().rotation }

    // there is a new aiming controller fetched every time the command is run.
    val aimingController by getOnceDuringRun {
        SuperPIDController(
            APRILTAG_AIM_PID,
            getInput = { Scalar(apriltagVision.bestTarget?.tx ?: 0.0) },
            target = Scalar(0.0),
            outputRange = Scalar(-0.5)..Scalar(0.5)
        )
    }


    fun canFindTarget(silenceWarnings: Boolean = false): Boolean {
        val bestTarget = apriltagVision.bestTarget

        return if (bestTarget == null){
            if (!silenceWarnings){
                println("NO TARGET FOUND!")
                NO_TARGET_FOUND_ALERT.active = true
            }
            false
        }else if (targetId != bestTarget.fiducialId){
            if (!silenceWarnings){
                val alertText = "An apriltag command can find an apriltag, " +
                        "but it does not match the id of $targetId. " +
                        "(Found id: " + bestTarget.fiducialId

                Alert.warning(text = alertText).active = true
                println(alertText)
            }
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

        return (aimingError in APRILTAG_AIM_PRECISION.allowableError).also{
            Logger.recordOutput("AimToLocation/hasFinishedAiming", it)
        }
    }

    fun getDistanceErrorToTag(): Distance {
        if (tagLocation.targetDistanceToTag == null) return 0.meters

        val distance = apriltagVision.robotToTargetDistance(tagPose.z) ?: return 0.meters // z is the tag height

        return (distance - tagLocation.targetDistanceToTag).also{
            Logger.recordOutput("AimToLocation/distanceTest", it.siValue)
        }
    }

    addRequirements(drivetrain, pivot)

    runOnce{
        apriltagVision.reset()
    }

    // runs the path following until tag within certain range, then cancels it
    // this ensures a smooth transition to aiming + getting within range
    if (followPathCommand != null){
        runUntil(
            {
                (canFindTarget(silenceWarnings = true) &&
                        getDistanceErrorToTag() < 0.4.meters &&
                        abs(drivetrain.heading - headingToFaceAprilTag) < 20.degrees)
            }, // we want to silence warnings here because during the pathing process, there might be other tags detected
            followPathCommand
        )
    }

    runOnce{
        // aims to the appropriate angle while strafing to the apriltag
        // this way, the lineup is appropriate.
        drivetrain.setRotationOverride(
            AimToAngleRotationOverride(
                headingToFaceAprilTag,
                ANGLE_TO_ROTATIONAL_VELOCITY_PID
            )
        )
    }

    // must be run in parallel in order to acheive higher performance
    runParallelUntilAllFinish{
        +pivot.setAngleCommand(tagLocation.pivotAngle)

        loopUntil({ (!canFindTarget() || hasFinishedAiming()) && getDistanceErrorToTag() <= Distance(0.01) }){
            Logger.recordOutput("AimToLocation/isAiming", true)
            drivetrain.swerveDrive(
                xPower = getDistanceErrorToTag().siValue * DISTANCE_REACH_KP,
                yPower = -aimingController.calculateOutput().siValue,
                rotationPower = 0.0,
                fieldRelative = false
            )
        }
    }

    onEnd{
        drivetrain.stop()
        Logger.recordOutput("AimToLocation/isAiming", false)
        drivetrain.removeRotationOverride()
    }
}