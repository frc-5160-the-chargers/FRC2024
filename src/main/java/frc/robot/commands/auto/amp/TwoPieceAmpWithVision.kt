package frc.robot.commands.auto.amp

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.DriveToTargetLocation
import frc.robot.commands.aimToAprilTag
import frc.robot.commands.auto.basicTaxi
import frc.robot.commands.runGroundIntake
import frc.robot.constants.CLOSED_LOOP_ROTATION_PID
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import kotlin.jvm.optionals.getOrNull


@Suppress("unused")
fun twoPieceAmpWithVision(
    apriltagDetector: AprilTagVisionPipeline,
    gamepieceDetector: ObjectVisionPipeline,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    fun stopSubsystems(){
        runOnce{
            drivetrain.removeRotationOverride()
            shooter.setPivotSpeed(0.0)
            shooter.setSpeed(0.0)
            groundIntake.spin(0.0)
        }
    }

    +onePieceAmp(shooter)

    runOnce{
        drivetrain.setRotationOverride(
            AimToObjectRotationOverride(
                gamepieceDetector,
                CLOSED_LOOP_ROTATION_PID
            )
        )
    }

    runParallelUntilOneFinishes{
        runSequentially{
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpGrab"))

            if (shooter.canDetectGamepieces){
                // rotation override set; no need for rotation output
                loopUntil( {shooter.hasGamepiece || gamepieceDetector.bestTarget == null} ){
                    drivetrain.velocityDrive(Velocity(-0.5), Velocity(0.0), AngularVelocity(0.0), fieldRelative = false)
                }
            }
        }

        +runGroundIntake(shooter, groundIntake)
    }

    stopSubsystems()

    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("2pAmpScore"), PATHFIND_CONSTRAINTS)

        +shooter.setAngleCommand(PivotAngle.AMP)
    }


    val tagId: Int = when (DriverStation.getAlliance().getOrNull()){
        null -> 0

        DriverStation.Alliance.Red -> DriveToTargetLocation.AMP.blueAllianceApriltagId

        DriverStation.Alliance.Blue -> DriveToTargetLocation.AMP.blueAllianceApriltagId
    }

    // performs necessary aiming
    +aimToAprilTag(
        tagId,
        OPEN_LOOP_TRANSLATION_PID,
        drivetrain = drivetrain,
        visionIO = apriltagDetector
    )

    loopFor(1.seconds){
        shooter.setSpeed(0.3)
        drivetrain.stop()
    }

    stopSubsystems()

    runParallelUntilAllFinish{
        +basicTaxi(
            drivetrain, shooter = shooter, groundIntake = groundIntake
        )

        +shooter.setAngleCommand(PivotAngle.IDLE)
    }

}