package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.NOTE_DETECTOR_PID
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.SpeakerAutoComponent
import frc.robot.commands.shootInSpeaker
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter


/**
 * A modular autonomous command for speaker-scoring autos with advanced path planning.
 */
fun speakerAutonomous(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
    noteDetector: ObjectVisionPipeline? = null,

    blueStartingPose: UnitPose2d,
    additionalComponents: List<SpeakerAutoComponent>
): Command = buildCommand {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetPose(blueStartingPose.flipWhenNeeded())
    }

    +shootInSpeaker(shooter, groundIntake, pivot, shooterSpinUpTime = 1.5.seconds)

    for (autoComponent in additionalComponents){
        val grabPathStartPose: UnitPose2d = autoComponent.grabPath.pathPoses.last().ofUnit(meters)
        var hasReceivedNote = false

        runOnce{
            hasReceivedNote = false
        }

        runParallelUntilFirstCommandFinishes{
            // parallel #1
            runSequentially{
                +AutoBuilder.followPath(autoComponent.grabPath)

                // drives out further in case the path missed the note
                if (noteDetector != null){
                    +pursueNote(
                        drivetrain, noteDetector,
                        endCondition = { hasReceivedNote }
                    ).withTimeout(2.0)
                }

                runOnce{
                    drivetrain.removeRotationOverride()
                }
            }

            // parallel #2
            loop{
                if (autoComponent.spinupShooterDuringGrabPath){
                    shooter.outtakeAtSpeakerSpeed()
                }else{
                    shooter.setIdle()
                }

                groundIntake.intake()

                if (groundIntake.hasNoteDetector){
                    if (groundIntake.hasNote){
                        hasReceivedNote = true
                    }
                }else{
                    if (noteDetector != null && noteDetector.bestTarget == null){
                        hasReceivedNote = true
                    }
                }
            }

            // parallel #3
            +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

            // parallel #4
            if (noteDetector != null){
                runSequentially{
                    // rotation override set is delayed as to prevent the drivetrain from aiming to a random note along the path.
                    // robotPose getter is a UnitPose2d; a wrapper with support for kmeasure units
                    waitUntil{ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE }

                    runOnce{
                        drivetrain.setRotationOverride(
                            AimToObjectRotationOverride(noteDetector, NOTE_DETECTOR_PID)
                        )
                    }
                }
            }
        }

        if (autoComponent.scorePath != null){
            runParallelUntilFirstCommandFinishes{
                +AutoBuilder.followPath(autoComponent.scorePath)

                +pivot.setAngleCommand(PivotAngle.SPEAKER)

                // just run shooting to bring the shooter up to speed
                loop{
                    shooter.outtakeAtSpeakerSpeed()
                    if (groundIntake.hasNoteDetector && groundIntake.hasNote){
                        groundIntake.intake()
                    }else{
                        groundIntake.setIdle()
                    }
                }
            }

            +shootInSpeaker(
                shooter, groundIntake, pivot,
                shooterSpinUpTime = 0.seconds
            )
        }
    }
}