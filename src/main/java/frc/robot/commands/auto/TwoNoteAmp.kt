package frc.robot.commands.auto


import com.batterystaple.kmeasure.units.seconds
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
        println("Hi")
        shooter.outtake(0.3)
    }


    +driveThenGroundIntakeToShooter(
        path = PathPlannerPath.fromPathFile("AmpGrabG2"),
        noteDetector = noteDetector,
        drivetrain, shooter, pivot, groundIntake
    )

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    +driveToLocation(
        target = FieldLocation.AMP,
        path = PathPlannerPath.fromPathFile("AmpScoreG2"),
        drivetrain, apriltagVision, pivot
    )

    loopFor(0.5.seconds){
        shooter.outtake(0.3)
    }

    +idleSubsystems(drivetrain, shooter, pivot, groundIntake)

    +basicTaxi(drivetrain)
}