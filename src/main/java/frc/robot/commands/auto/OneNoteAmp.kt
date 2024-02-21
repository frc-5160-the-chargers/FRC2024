package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.auto.components.AmpAutoEndAction
import frc.robot.commands.auto.components.ferryNotes
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

fun oneNoteAmp(
    shooter: Shooter,
    pivot: Pivot,
): Command = buildCommand("One Note Amp") {
    +pivot.setAngleCommand(PivotAngle.AMP)

    loopFor(0.2.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        shooter.setIdle()
    }
}


fun oneNoteAmp(
    noteDetector: ObjectVisionPipeline? = null,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
    endAction: AmpAutoEndAction
): Command = buildCommand(name = "One Note Amp"){
    +oneNoteAmp(shooter, pivot)

    when (endAction){
        AmpAutoEndAction.STOW_PIVOT -> {
            +pivot.setAngleCommand(PivotAngle.STOWED)
        }

        AmpAutoEndAction.TAXI -> {
            runParallelUntilAllFinish{
                +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))

                +pivot.setAngleCommand(PivotAngle.STOWED)
            }
        }

        AmpAutoEndAction.FERRY -> {
            +ferryNotes(noteDetector, drivetrain, pivot, groundIntake, 3)
        }
    }
}

