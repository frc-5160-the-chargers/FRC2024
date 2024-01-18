package frc.robot.commands.auto.amp

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.MLVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.grabGamepiece
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
    +shooter.scoreAmpCommand()

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