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
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.Shooter


@Suppress("unused")
fun twoPieceAmpWithVision(
    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    +onePieceAmp(shooter)

    runOnce{
        drivetrain.poseEstimator.resetToPathplannerTrajectory("2pAmpGrab")
    }

    +grabGamepiece(
        path = PathPlannerPath.fromPathFile("2pAmpGrab"),
        noteDetector = noteDetector,
        drivetrain, shooter, groundIntake
    )

    +driveToLocation(
        target = FieldLocation.AMP,
        path = PathPlannerPath.fromPathFile("2pAmpScore"),
        drivetrain, apriltagVision, shooter
    )

    loopFor(1.seconds){
        shooter.outtake(0.3)
    }

    runOnce{
        shooter.setIdle()
    }

    +basicTaxi(
        drivetrain, shooter = shooter, groundIntake = groundIntake
    )
}