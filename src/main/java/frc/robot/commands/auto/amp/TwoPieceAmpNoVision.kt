package frc.robot.commands.auto.amp

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.auto.basicTaxi
import frc.robot.commands.runGroundIntake
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

@Suppress("unused")
fun twoPieceAmpNoVision(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    addRequirements(drivetrain, shooter, groundIntake)

    +onePieceAmp(shooter)

    runParallelUntilOneFinishes{
        +runGroundIntake(
            shooter,
            groundIntake,
            indefinite = true
        )


        runSequentially{
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpGrab"))

            if (shooter.canDetectGamepieces){
                loopUntil( {shooter.hasGamepiece} ){
                    drivetrain.swerveDrive(-0.15,0.0, 0.0, fieldRelative = false)
                }
            }
        }
    }



    runParallelUntilAllFinish{
        +AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("2pAmpScore"), PATHFIND_CONSTRAINTS)

        +shooter.setAngleCommand(PivotAngle.AMP)
    }

    loopFor(1.seconds){
        shooter.setSpeed(0.3)
    }

    runOnce{
        shooter.setSpeed(0.0)
    }

    +basicTaxi(
        drivetrain, shooter = shooter, groundIntake = groundIntake
    )
}