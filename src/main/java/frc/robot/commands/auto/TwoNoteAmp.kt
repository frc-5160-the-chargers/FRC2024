package frc.robot.commands.auto


import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.*
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
): Command = buildCommand(name = "Two piece amp(with vision)", logIndividualCommands = true) {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("AmpGrabG2", useHolonomicPose = true)
    }

    +pivot.setAngleCommand(PivotAngle.AMP)

    loopFor(0.5.seconds, shooter){
        shooter.outtake(0.3)
    }

    +driveToNoteAndIntake(
        noteDetector,
        drivetrain, shooter, pivot, groundIntake,
        path = PathPlannerPath.fromPathFile("AmpGrabG2"),
    )

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    fun passToShooter(){
        // sets pivot angle to ground intake handoff
        pivot.setAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
        groundIntake.passToShooter(shooter)
    }

    // during pathing, we want to pass the note
    runParallelUntilFirstCommandFinishes{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpScoreG2"))

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
    +driveToLocation(
        drivetrain, apriltagVision, pivot,
        target = FieldLocation.AMP
    )

    loopFor(0.5.seconds){
        shooter.outtake(0.3)
    }

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    runParallelUntilAllFinish{
        +basicTaxi(drivetrain)

        +pivot.setAngleCommand(PivotAngle.STOWED)
    }
}