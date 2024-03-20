package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.robot.AMP_AUTO_STARTING_POSE_BLUE
import frc.robot.commands.auto.components.AmpAutoScoreComponent
import frc.robot.commands.followPathOptimal
import frc.robot.commands.runGroundIntake
import frc.robot.commands.passSerializedNote
import frc.robot.commands.shootInAmp
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

/**
 * An Amp autonomous command without vision cameras.
 */
fun noVisionAmpAutonomous(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    taxiAtEnd: Boolean = false,
    additionalComponents: List<AmpAutoScoreComponent> = listOf() // used to control further notes pursued.
): Command = buildCommand {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetPose(
            AMP_AUTO_STARTING_POSE_BLUE.flipWhenNeeded()
        )
    }

    +AutoBuilder.followPath(
        PathPlannerPath.fromPathFile("DriveToAmp")
    )

    +shootInAmp(shooter, pivot)

    for (autoComponent in additionalComponents){
        runParallelUntilFirstCommandFinishes{
            // parallel #1
            runSequentially{
                +followPathOptimal(drivetrain, autoComponent.grabPath)

                waitFor(1.seconds)
            }

            // parallel #2
            runSequentially{
                +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                +runGroundIntake(groundIntake, shooter)
            }
        }

        runParallelUntilFirstCommandFinishes{
            +followPathOptimal(drivetrain, autoComponent.scorePath)

            runSequentially{
                +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                +passSerializedNote(groundIntake, shooter)
            }
        }

        +shootInAmp(shooter, pivot)
    }

    runParallelUntilAllFinish{
        if (taxiAtEnd){
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))
        }

        +pivot.setAngleCommand(PivotAngle.STOWED)
    }
}