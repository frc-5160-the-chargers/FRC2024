package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.SpeakerAutoScoreComponent
import frc.robot.commands.auto.components.SpeakerAutoStartingPose
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInSpeaker
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import frc.robot.controls.rotationoverride.getSpeakerRotationOverride
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

private val CLOSE_RANGE_SPEAKER_SHOOT_TIMEOUT = 0.8.seconds
private val FAR_RANGE_SPEAKER_SHOOT_TIMEOUT = 1.0.seconds

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
    val noteRotationOverride = getNoteRotationOverride(noteDetector)

    runOnce{
        drivetrain.poseEstimator.resetPose(startingPose.pose.flipWhenNeeded())
    }

    +shootInSpeaker(shooter, groundIntake, pivot)

    for (autoComponent in additionalComponents){
        val grabPathStartPose: UnitPose2d = autoComponent.grabPath.pathPoses.last().ofUnit(meters)

        runParallelUntilFirstCommandFinishes{
            // parallel #1
            runSequentially{
                +AutoBuilder.followPath(autoComponent.grabPath)

                // drives out further in case the path missed the note
                +pursueNote(
                    drivetrain, noteDetector,
                    endCondition = { groundIntake.hasNote },
                    setRotationOverride = false
                ).withTimeout(2.0)
            }

            // parallel #2
            runSequentially{
                // rotation override set is delayed as to prevent the drivetrain from aiming to a random note
                // along the path.
                waitUntil{ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE }

                runOnce{
                    drivetrain.setRotationOverride(noteRotationOverride)
                }

                +runGroundIntake(groundIntake, shooter)
            }
        }

        runOnce{
            drivetrain.setRotationOverride(speakerRotationOverride)
        }

        if (autoComponent.scorePath != null){
            runParallelUntilFirstCommandFinishes{
                +AutoBuilder.followPath(autoComponent.scorePath)

                if (autoComponent.shooterShouldStartDuringPath){
                    loop{
                        // just run shooting to bring the shooter up to speed
                        shooter.outtakeAtSpeakerSpeed()
                        groundIntake.setConveyorVoltage(-2.volts) // sets a small voltage so that note won't entirely go into shooter
                    }
                }
            }
        }

        if (autoComponent.shouldShootOnEnd){
            +shootInSpeaker(
                shooter, groundIntake, pivot,
                shooterSpinUpTime = if (autoComponent.shooterShouldStartDuringPath) 0.seconds else 0.3.seconds
            )
        }

        runOnce{
            drivetrain.removeRotationOverride()
        }
    }
}