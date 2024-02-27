package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.SpeakerAutoScoreComponent
import frc.robot.commands.auto.components.SpeakerAutoStartingPose
import frc.robot.commands.followPathOptimal
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInSpeaker
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import frc.robot.controls.rotationoverride.getSpeakerRotationOverride
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

private val CLOSE_RANGE_SPEAKER_SHOOT_TIMEOUT = 0.7.seconds
private val FAR_RANGE_SPEAKER_SHOOT_TIMEOUT = 0.9.seconds

/**
 * A modular autonomous command for speaker-side autos.
 */
fun speakerAutonomous(
    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    startingPose: SpeakerAutoStartingPose,
    additionalComponents: List<SpeakerAutoScoreComponent>
): Command = buildCommand {
    val speakerRotationOverride = getSpeakerRotationOverride(apriltagVision)

    runOnce{
        drivetrain.poseEstimator.resetPose(startingPose.pose)
    }

    +shootInSpeaker(shooter, groundIntake, pivot, timeout = CLOSE_RANGE_SPEAKER_SHOOT_TIMEOUT)

    for (autoComponent in additionalComponents){
        val grabPathStartPose: UnitPose2d = autoComponent.grabPath.pathPoses.last().ofUnit(meters)

        runParallelUntilFirstCommandFinishes{
            // parallel #1
            runSequentially{
                +followPathOptimal(drivetrain, autoComponent.grabPath)

                // drives out further in case the path missed the note
                +pursueNote(drivetrain, noteDetector)

                waitFor(0.2.seconds) // waits a little so that the note can fully go in
            }

            // parallel #2
            runSequentially{
                loopUntil({ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE }){
                    pivot.setAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
                }

                runOnce{
                    drivetrain.setRotationOverride(getNoteRotationOverride(noteDetector))
                }

                +runGroundIntake(groundIntake, pivot, shooter)
            }
        }

        runOnce{
            drivetrain.setRotationOverride(speakerRotationOverride)
        }

        if (autoComponent.scorePath != null){
            +followPathOptimal(drivetrain, autoComponent.scorePath)
        }

        waitUntil{speakerRotationOverride.atSetpoint}

        if (autoComponent.shouldShootOnEnd){
            +shootInSpeaker(
                shooter, groundIntake, pivot,
                timeout = if (autoComponent.isFarRangeShot){
                    FAR_RANGE_SPEAKER_SHOOT_TIMEOUT
                }else{
                    CLOSE_RANGE_SPEAKER_SHOOT_TIMEOUT
                }
            )
        }

        runOnce{
            drivetrain.removeRotationOverride()
        }
    }
}