package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.NOTE_DETECTOR_PID
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.AmpAutoComponent
import frc.robot.commands.auto.components.AmpAutoTaxiMode
import frc.robot.commands.auto.components.AutoStartingPose
import frc.robot.commands.passSerializedNote
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInAmp
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

/**
 * An Amp autonomous command without vision cameras.
 */
fun ampAutonomous(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
    noteDetector: ObjectVisionPipeline? = null,

    additionalComponents: List<AmpAutoComponent> = listOf(), // used to control further notes pursued.
    taxiMode: AmpAutoTaxiMode = AmpAutoTaxiMode.NO_TAXI
): Command = buildCommand {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runParallelUntilAllFinish{
        runSequentially{
            runOnce{
                drivetrain.poseEstimator.resetPose(
                    // flipWhenNeeded is an extension function of a UnitPose2d
                    AutoStartingPose.AMP_BLUE.flipWhenNeeded()
                )
            }

            waitFor(0.4.seconds)

            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToAmp"))
        }

        +pivot.setAngleCommand(PivotAngle.AMP)
    }

    +shootInAmp(shooter, pivot)

    for (autoComponent in additionalComponents) {
        // starts ground intake a little before path
        if (autoComponent.groundIntakePreSpinupTime != null) {
            +runGroundIntake(groundIntake, shooter, timeout = autoComponent.groundIntakePreSpinupTime)
        }

        runParallelUntilFirstCommandFinishes {
            // parallel #1
            runSequentially {
                +AutoBuilder.followPath(autoComponent.grabPath)

                if (noteDetector != null){
                    +pursueNote(
                        drivetrain, noteDetector,
                        endCondition = { if (RobotBase.isSimulation()) noteDetector.bestTarget != null else groundIntake.hasNote }
                    )
                }

                waitFor(0.5.seconds)
            }

            // parallel #2
            +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

            // parallel #3
            +runGroundIntake(groundIntake, shooter)

            // parallel #4
            if (noteDetector != null){
                // rotation override set is delayed as to prevent the drivetrain from aiming to a random note
                // along the path.
                runSequentially{
                    val grabPathStartPose = autoComponent.grabPath.pathPoses.last().ofUnit(meters).flipWhenNeeded()

                    waitUntil{ drivetrain.poseEstimator.robotPose.distanceTo(grabPathStartPose) < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE }

                    runOnce{
                        drivetrain.setRotationOverride(
                            AimToObjectRotationOverride(noteDetector, NOTE_DETECTOR_PID)
                        )
                    }
                }
            }
        }

        if (noteDetector != null){
            runOnce{
                drivetrain.removeRotationOverride()
            }
        }

        runParallelUntilAllFinish {
            +AutoBuilder.followPath(autoComponent.scorePath)

            runSequentially {
                +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                +passSerializedNote(groundIntake, shooter)
            }
        }

        if (autoComponent.shouldScore) +shootInAmp(shooter, pivot)
    }

    +pivot.setAngleCommand(PivotAngle.STOWED)

    when (taxiMode){
        AmpAutoTaxiMode.TAXI_SHORT -> {
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxiShort"))
        }

        AmpAutoTaxiMode.TAXI_LONG -> {
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))
        }

        AmpAutoTaxiMode.NO_TAXI -> {}
    }
}