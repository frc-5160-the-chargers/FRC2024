package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.commands.auto.components.AutoStartingPose
import frc.robot.commands.runGroundIntake
import frc.robot.commands.shootInSpeaker
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter

fun basicTaxi(drivetrain: EncoderHolonomicDrivetrain): Command =
    buildCommand{
        loopFor(5.seconds, drivetrain){
            drivetrain.swerveDrive(0.2,0.0,0.0, fieldRelative = false)
        }

        onEnd{
            drivetrain.stop()
        }
    }


fun onePieceSpeakerAndTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntakeSerializer,
    pivot: Pivot,
    blueStartingPose: UnitPose2d = AutoStartingPose.SPEAKER_CENTER_BLUE,
): Command = buildCommand {
    runOnce{
        drivetrain.poseEstimator.resetPose(blueStartingPose.flipWhenNeeded())
    }

    +shootInSpeaker(shooter, groundIntake, pivot, shooterSpinUpTime = 2.seconds)

    +basicTaxi(drivetrain)
}

fun twoPieceSpeakerAndTaxi(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntakeSerializer,
    pivot: Pivot
): Command = buildCommand {
    addRequirements(drivetrain, groundIntake, pivot, shooter)

    runOnce{
        drivetrain.poseEstimator.resetPose(AutoStartingPose.SPEAKER_CENTER_BLUE.flipWhenNeeded())
    }

    +shootInSpeaker(shooter, groundIntake, pivot)

    runParallelUntilFirstCommandFinishes{
        loopFor(2.seconds){
            drivetrain.swerveDrive(0.2, 0.0, 0.0, fieldRelative = false)
        }

        +runGroundIntake(groundIntake, shooter)
    }


    loopFor(2.seconds){
        drivetrain.swerveDrive(-0.2, 0.0, 0.0, fieldRelative = false)
    }

    runOnce{
        drivetrain.stop()
    }

    +shootInSpeaker(shooter, groundIntake, pivot)

    +basicTaxi(drivetrain)
}


