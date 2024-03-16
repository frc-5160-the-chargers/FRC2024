package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.SpeakerAutoScoreComponent
import frc.robot.commands.auto.components.SpeakerAutoStartingPose
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInSpeaker
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter


/**
 * A modular autonomous command for speaker-side autos.
 */
fun speakerAutonomous(
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    startingPose: SpeakerAutoStartingPose,
    additionalComponents: List<SpeakerAutoScoreComponent>
): Command = buildCommand {
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
                    endCondition = { if (isReal()) groundIntake.hasNote else noteDetector.bestTarget == null },
                    setRotationOverride = false
                ).withTimeout(2.0)
            }

            // parallel #2
            runSequentially{
                // rotation override set is delayed as to prevent the drivetrain from aiming to a random note
                // along the path.
                // robotPose getter is a UnitPose2d; a wrapper with support for kmeasure units
                waitUntil{ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE }

                runOnce{
                    drivetrain.setRotationOverride(noteRotationOverride)
                }

                +runGroundIntake(groundIntake, shooter)
            }
        }

        runOnce{
            drivetrain.removeRotationOverride()
        }

        if (autoComponent.scorePath != null){
            runParallelUntilFirstCommandFinishes{
                +AutoBuilder.followPath(autoComponent.scorePath)

                if (autoComponent.shooterShouldStartDuringPath){
                    loop{
                        // just run shooting to bring the shooter up to speed
                        shooter.outtakeAtSpeakerSpeed()
                    }
                }
            }

            +shootInSpeaker(
                shooter, groundIntake, pivot,
                shooterSpinUpTime = if (autoComponent.shooterShouldStartDuringPath) 0.seconds else 0.5.seconds
            )
        }
    }
}