package frc.robot.commands.auto


import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.FieldLocation
import frc.robot.commands.driveToLocation
import frc.robot.commands.grabGamepiece
import frc.robot.commands.shootInAmp
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter


@Suppress("unused")
fun twoPieceAmpWithVision(
    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    +shootInAmp(pivot, shooter,0.3, 0.5.seconds)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("AmpGrabG2")
    }

    +grabGamepiece(
        path = PathPlannerPath.fromPathFile("AmpGrabG2"),
        noteDetector = noteDetector,
        drivetrain, pivot, shooter, groundIntake
    )

    +driveToLocation(
        target = FieldLocation.AMP,
        path = PathPlannerPath.fromPathFile("AmpScoreG2"),
        drivetrain, apriltagVision, pivot
    )

    +shootInAmp(pivot, shooter,0.3, 0.5.seconds)

    +basicTaxi(
        drivetrain, shooter = shooter, groundIntake = groundIntake
    )
}