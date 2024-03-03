package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.AMP_AUTO_STARTING_POSE_BLUE
import frc.robot.commands.*
import frc.robot.commands.aiming.AprilTagLocation
import frc.robot.commands.aiming.alignToAprilTag
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.AmpAutoScoreComponent
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import kotlin.jvm.optionals.getOrNull



/**
 * A modular amp autonomous command, used for all of our amp autos.
 */
fun ampAutonomous(
    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    additionalComponents: List<AmpAutoScoreComponent> = listOf(), // used to control further notes pursued.
    taxiAtEnd: Boolean = false,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetPose(AMP_AUTO_STARTING_POSE_BLUE.flipWhenNeeded())
    }

    loopFor(0.2.seconds){
        if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
            drivetrain.swerveDrive(0.13, 0.13, 0.0, fieldRelative = false)
        }else{
            drivetrain.swerveDrive(0.13, -0.13, 0.0, fieldRelative = false)
        }
    }

    +alignToAprilTag(
        drivetrain,
        apriltagVision,
        pivot,
        AprilTagLocation.AMP
    )

    loopFor(0.3.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        shooter.setIdle()
    }

    for (autoComponent in additionalComponents){
        val grabPathStartPose = autoComponent.grabPath.pathPoses.last().ofUnit(meters).flipWhenNeeded()

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
                // rotation override set is delayed as to prevent the drivetrain from aiming to a random note
                // along the path.
                runUntil(
                    { drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE },
                    pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)
                )

                runOnce{
                    drivetrain.setRotationOverride(getNoteRotationOverride(noteDetector))
                }

                +runGroundIntake(groundIntake, pivot, shooter)
            }
        }

        runOnce{
            drivetrain.removeRotationOverride()
        }

        when (autoComponent.type){
            AmpAutoScoreComponent.Type.FERRY_NOTE -> {
                runParallelUntilAllFinish{
                    +followPathOptimal(drivetrain, autoComponent.grabPath)

                    +passSerializedNote(groundIntake, shooter, pivot)
                }

                loopFor(0.5.seconds){
                    groundIntake.outtake()
                }

                runOnce{
                    groundIntake.setIdle()
                }
            }

            AmpAutoScoreComponent.Type.SCORE_NOTE -> {
                +alignToAprilTag(
                    drivetrain, apriltagVision, pivot,
                    AprilTagLocation.AMP,
                    followPathCommand = -runParallelUntilAllFinish{
                        +followPathOptimal(drivetrain, autoComponent.scorePath)

                        +passSerializedNote(groundIntake, shooter, pivot)
                    } // negative sign removes the block from the command builder, as the parallel command is added to the builder by default
                )

                loopFor(0.3.seconds){
                    shooter.outtake(0.5)
                }

                runOnce{
                    shooter.setIdle()
                }
            }
        }
    }

    runParallelUntilAllFinish{
        if (taxiAtEnd){
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))
        }

        +pivot.setAngleCommand(PivotAngle.STOWED)
    }
}