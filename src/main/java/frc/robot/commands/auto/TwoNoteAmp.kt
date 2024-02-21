package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ANGLE_TO_ROTATIONAL_VELOCITY_PID
import frc.robot.commands.*
import frc.robot.commands.auto.components.AmpAutoEndAction
import frc.robot.commands.auto.components.ferryNotes
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

@Suppress("unused")
fun twoNoteAmp(
    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    endAction: AmpAutoEndAction
): Command = buildCommand(name = "Two note amp(with vision)", logIndividualCommands = true) {
    fun passToShooter(){
        // sets pivot angle to ground intake handoff
        pivot.setAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
        groundIntake.passToShooter(shooter)
    }

    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("AmpGrabG1", useHolonomicPose = true)
    }

    +oneNoteAmp(shooter, pivot)

    +driveToNoteAndIntake(
        noteDetector,
        drivetrain, pivot, groundIntake,
        path = PathPlannerPath.fromPathFile("AmpGrabG1"),
    )

    runOnce{
        groundIntake.setIdle()
        drivetrain.stop()
        drivetrain.setRotationOverride(
            AimToAngleRotationOverride(
                90.degrees,
                ANGLE_TO_ROTATIONAL_VELOCITY_PID
            )
        )
    }

    // during pathing, we want to pass the note
    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpScoreG1"))

        if (shooter.hasBeamBreakSensor){
            loopUntil({shooter.hasNote}){
                passToShooter()
            }
        }else{
            loopFor(0.7.seconds){
                passToShooter()
            }
        }
    }

    // already sets pivot angle; so we dont need to explicitly set it
    +driveToLocation(
        drivetrain, apriltagVision, pivot,
        target = FieldLocation.AMP
    )

    loopFor(0.2.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        drivetrain.removeRotationOverride()
        shooter.setIdle()
    }

    loopFor(0.2.seconds){
        drivetrain.swerveDrive(-0.2,0.0,0.0, fieldRelative = false)
    }

    when (endAction){
        AmpAutoEndAction.STOW_PIVOT -> {
            +pivot.setAngleCommand(PivotAngle.STOWED)
        }

        AmpAutoEndAction.TAXI -> {
            runParallelUntilAllFinish{
                +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))

                +pivot.setAngleCommand(PivotAngle.STOWED)
            }
        }

        AmpAutoEndAction.FERRY -> {
            +ferryNotes(noteDetector, drivetrain, pivot, groundIntake, 2)
        }
    }

    onEnd{
        drivetrain.removeRotationOverride()
    }
}



@Suppress("unused")
fun twoNoteAmpWithoutVision(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    endAction: AmpAutoEndAction
): Command = buildCommand(name = "Two note amp(no vision)", logIndividualCommands = true) {
    fun passToShooter(){
        // sets pivot angle to ground intake handoff
        pivot.setAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
        groundIntake.passToShooter(shooter)
    }

    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("AmpGrabG1", useHolonomicPose = true)
    }

    +oneNoteAmp(shooter, pivot)

    runParallelUntilFirstCommandFinishes{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpGrabG1"))

        loop{
            groundIntake.intake(pivot)
        }
    }

    runOnce{
        groundIntake.setIdle()
        drivetrain.stop()
        drivetrain.setRotationOverride(
            AimToAngleRotationOverride(
                90.degrees,
                ANGLE_TO_ROTATIONAL_VELOCITY_PID
            )
        )
    }

    // during pathing, we want to pass the note
    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpScoreG1"))

        if (shooter.hasBeamBreakSensor){
            loopUntil({shooter.hasNote}){
                passToShooter()
            }
        }else{
            loopFor(0.5.seconds){
                passToShooter()
            }
        }
    }

    // already sets pivot angle; so we dont need to explicitly set it
    +pivot.setAngleCommand(PivotAngle.AMP)

    loopFor(0.2.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        drivetrain.removeRotationOverride()
        shooter.setIdle()
    }

    loopFor(0.2.seconds){
        drivetrain.swerveDrive(-0.2,0.0,0.0, fieldRelative = false)
    }

    when (endAction){
        AmpAutoEndAction.STOW_PIVOT -> {
            +pivot.setAngleCommand(PivotAngle.STOWED)
        }

        AmpAutoEndAction.TAXI -> {
            runParallelUntilAllFinish{
                +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))

                +pivot.setAngleCommand(PivotAngle.STOWED)
            }
        }

        AmpAutoEndAction.FERRY -> {
            +ferryNotes(noteDetector = null, drivetrain, pivot, groundIntake, 2)
        }
    }

    onEnd{
        drivetrain.removeRotationOverride()
    }
}