package frc.robot.commands.auto.speaker
/*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.pathplannerextensions.PathPlannerPaths
import frc.robot.constants.OPEN_LOOP_ROTATION_PID
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.Shooter



fun fivePieceSpeakerBeta(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,

    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline
): Command = buildCommand {
    val trajGroupName = "5pAutoLeft"

    val paths = PathPlannerPaths.fromChoreoTrajectoryGroup(trajGroupName)

    addRequirements(drivetrain, shooter, groundIntake)


    fun intakeGamepiecePath(path: PathPlannerPath){
        runOnce{
            drivetrain.setRotationOverride(
                AimToObjectRotationOverride(
                    noteDetector,
                    OPEN_LOOP_ROTATION_PID
                )
            )
        }

        +AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS)

        loopWhile( { shooter.canDetectGamepieces && !shooter.hasGamepiece }){
            drivetrain.swerveDrive(

            )
        }
    }


    runOnce {
        drivetrain.poseEstimator.resetToChoreoTrajectory(trajGroupName)
    }


    paths.forEach{
        +AutoBuilder.followPath(it)
    }

}

 */