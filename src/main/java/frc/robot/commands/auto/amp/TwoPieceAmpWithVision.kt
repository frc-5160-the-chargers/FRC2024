package frc.robot.commands.auto.amp

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.AimingConfig
import frc.robot.commands.apriltag.AimToTargetGoal
import frc.robot.commands.apriltag.aimToTarget
import frc.robot.commands.grabGamepiece
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter


@Suppress("unused")
fun twoPieceAmpWithVision(
    aimingPID: PIDConstants = OPEN_LOOP_TRANSLATION_PID,
    apriltagDetector: AprilTagVisionPipeline,
    gamepieceDetector: ObjectVisionPipeline,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    +onePieceAmp(shooter)

    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpGrab"))

        runSequentially{
            runOnce{
                gamepieceDetector.requireAndReset()
                apriltagDetector.requireAndReset()
            }

            +shooter.setAngleCommand(PivotAngle.IDLE)
        }
    }

    +grabGamepiece(
        aimingConfig = AimingConfig(
            gamepieceDetector,
            aimingPID
        ),
        drivetrain = drivetrain,
        shooter = shooter,
        groundIntake = groundIntake,
    )

    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpScore"))

        +shooter.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)
    }

    +aimToTarget(
        AimToTargetGoal.AMP,
        aimingPID,
        pathfind = false,
        drivetrain,
        apriltagDetector,
        shooter
    )

    runOnce{
        apriltagDetector.removeRequirement()
        gamepieceDetector.removeRequirement()
        drivetrain.stop()
    }

    loopFor(1.seconds){
        shooter.spin(0.3)
    }

    runOnce{
        shooter.spin(0.0)
        shooter.setPivotPercentOut(0.0)
        groundIntake.spin(0.0)
    }
}