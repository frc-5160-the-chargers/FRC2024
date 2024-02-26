package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.commands.auto.components.AmpAutoScoreComponent
import frc.robot.commands.followPathOptimal
import frc.robot.commands.intakeNoteFromGround
import frc.robot.commands.passSerializedNote
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

    addRequirements(drivetrain, shooter, pivot)

    runOnce{
        drivetrain.poseEstimator.resetPose(
            UnitPose2d(1.4.meters, 7.3.meters, 90.degrees).flipWhenNeeded()
        )
    }

    runParallelUntilAllFinish{
        +AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("DriveToAmp")
        )

        +pivot.setAngleCommand(PivotAngle.AMP)
    }

    loopFor(0.2.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        shooter.setIdle()
    }


    for (autoComponent in additionalComponents){
        runParallelUntilFirstCommandFinishes{
            // parallel #1
            +followPathOptimal(drivetrain, autoComponent.grabPath)

            // parallel #2
            +intakeNoteFromGround(groundIntake, pivot, shooter)
        }

        runParallelUntilAllFinish{
            +followPathOptimal(drivetrain, autoComponent.scorePath)

            +passSerializedNote(groundIntake, shooter, pivot)
        }

        loopFor(0.2.seconds){
            shooter.outtake(0.5)
        }

        runOnce{
            shooter.setIdle()
        }
    }

    runParallelUntilAllFinish{
        if (taxiAtEnd){
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))
        }

        +pivot.setAngleCommand(PivotAngle.STOWED)
    }
}