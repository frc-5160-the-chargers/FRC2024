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
import frc.robot.commands.*
import frc.robot.commands.aiming.AprilTagLocation
import frc.robot.commands.aiming.alignToAprilTag
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.AmpAutoComponent
import frc.robot.commands.auto.components.AutoStartingPose
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

    additionalComponents: List<AmpAutoComponent> = listOf(), // used to control further notes pursued.
    taxiAtEnd: Boolean = false,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetPose(AutoStartingPose.AMP_BLUE.flipWhenNeeded())
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

    +shootInAmp(shooter, pivot)

    for (autoComponent in additionalComponents){
        val grabPathStartPose = autoComponent.grabPath.pathPoses.last().ofUnit(meters).flipWhenNeeded()

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
                runUntil(
                    { drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE },
                    pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)
                )

                runOnce{
                    drivetrain.setRotationOverride(getNoteRotationOverride(noteDetector))
                }

                +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                +runGroundIntake(groundIntake, shooter)
            }
        }

        runOnce{
            drivetrain.removeRotationOverride()
        }

        when (autoComponent.type){
            AmpAutoComponent.Type.FERRY_NOTE -> {
                runParallelUntilAllFinish{
                    +AutoBuilder.followPath(autoComponent.grabPath)

                    runSequentially{
                        +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                        +passSerializedNote(groundIntake, shooter)
                    }
                }

                +shootInAmp(shooter, pivot)
            }

            AmpAutoComponent.Type.SCORE_NOTE -> {
                +alignToAprilTag(
                    drivetrain, apriltagVision, pivot,
                    AprilTagLocation.AMP,
                    followPathCommand = -runParallelUntilAllFinish{
                        +AutoBuilder.followPath(autoComponent.scorePath)

                        runSequentially{
                            +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                            +passSerializedNote(groundIntake, shooter)
                        }
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