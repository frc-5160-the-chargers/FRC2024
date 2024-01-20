package frc.robot.commands.auto.amp

/*
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.MLVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.apriltag.DriveToTargetGoal
import frc.robot.commands.apriltag.driveToTarget
import frc.robot.commands.gamepiece.grabGamepiece
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.Shooter





fun twoPieceAmpBasic(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
    gamepieceDetector: MLVisionPipeline,
    apriltagDetector: AprilTagVisionPipeline,
    aimingPID: PIDConstants
): Command = buildCommand {
    +driveToTarget(
        DriveToTargetGoal.AMP,
        pathfind = false,

        drivetrain = drivetrain, shooter = shooter, visionIO = apriltagDetector
    )

    loopFor(1.seconds, shooter){
        shooter.spin(0.3)
    }

    runOnce{
        gamepieceDetector.require()
        gamepieceDetector.reset()
    }

    loopUntil( {gamepieceDetector.bestTarget != null}, drivetrain){
        drivetrain.swerveDrive(0.2,0.0,0.0)
    }

    runOnce{
        drivetrain.stop()
        gamepieceDetector.removeRequirement()
    }

    +grabGamepiece(drivetrain, shooter, groundIntake, gamepieceDetector, aimingPID)

    runOnce{
        apriltagDetector.require()
        apriltagDetector.reset()
    }

    loopUntil ( {apriltagDetector.bestTarget != null}, drivetrain){
        drivetrain.swerveDrive(-0.2,0.0,0.0)
    }

    runOnce{
        apriltagDetector.removeRequirement()
    }
}

 */