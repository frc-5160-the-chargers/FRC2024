package frc.robot.commands.auto.amp

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.grabGamepiece
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

@Suppress("unused")
fun twoPieceAmpNoVision(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    groundIntake: GroundIntake,
): Command = buildCommand {
    +onePieceAmp(shooter)

    runParallelUntilAllFinish{
        +shooter.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpGrab"))
    }

    +grabGamepiece(
        drivetrain = drivetrain,
        shooter = shooter,
        groundIntake = groundIntake,
    )

    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pAmpScore"))

        +shooter.setAngleCommand(PivotAngle.IDLE)
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