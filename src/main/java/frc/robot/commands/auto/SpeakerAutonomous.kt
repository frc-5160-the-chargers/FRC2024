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
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_SPINUP
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.SpeakerAutoComponent
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInSpeaker
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter


/**
 * A modular autonomous command for speaker-scoring autos with advanced path planning.
 */
fun speakerAutonomous(
    noteDetector: ObjectVisionPipeline? = null,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    blueStartingPose: UnitPose2d,
    additionalComponents: List<SpeakerAutoComponent>
): Command = buildCommand {
    val noteRotationOverride = if (noteDetector != null){
        getNoteRotationOverride(noteDetector)
    }else{
        null
    }

    runOnce{
        drivetrain.poseEstimator.resetPose(blueStartingPose.flipWhenNeeded())
    }

    +shootInSpeaker(shooter, groundIntake, pivot)

    for (autoComponent in additionalComponents){
        val grabPathStartPose: UnitPose2d = autoComponent.grabPath.pathPoses.last().ofUnit(meters)

        runParallelUntilFirstCommandFinishes{
            // parallel #1
            runSequentially{
                +AutoBuilder.followPath(autoComponent.grabPath)

                // drives out further in case the path missed the note
                if (noteDetector != null){
                    +pursueNote(
                        drivetrain, noteDetector,
                        endCondition = { if (isReal()) groundIntake.hasNote else noteDetector.bestTarget == null },
                        setRotationOverride = false
                    ).withTimeout(2.0)
                }
            }

            // parallel #2
            runSequentially{
                if (noteRotationOverride != null){
                    // rotation override set is delayed as to prevent the drivetrain from aiming to a random note
                    // along the path.
                    // robotPose getter is a UnitPose2d; a wrapper with support for kmeasure units
                    waitUntil{ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE }

                    runOnce{
                        drivetrain.setRotationOverride(noteRotationOverride)
                    }
                }

                +runGroundIntake(groundIntake, shooter)
            }
        }

        runOnce{
            drivetrain.removeRotationOverride()
        }

        if (autoComponent.scorePath != null){
            runParallelUntilFirstCommandFinishes{
                // gives at least 2 seconds for the shooter to spin up
                runParallelUntilAllFinish{
                    +AutoBuilder.followPath(autoComponent.scorePath)

                    waitFor(2.seconds)
                }

                runSequentially{
                    waitUntil{ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_SPINUP }

                    loop{
                        // just run shooting to bring the shooter up to speed
                        shooter.outtakeAtSpeakerSpeed()
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