package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.*
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

// We don't have time for other end actions; thus, none are offered
fun threeNoteAmp(
    apriltagVision: AprilTagVisionPipeline,
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
): Command = buildCommand {
    +twoNoteAmp(apriltagVision, noteDetector, drivetrain, shooter, pivot, groundIntake, endAction = null)

    +driveToNoteAndIntake(noteDetector, drivetrain, pivot, groundIntake, path = PathPlannerPath.fromPathFile("AmpGrabG3"))

    runParallelUntilAllFinish{
        +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpScoreG3"))

        if (shooter.hasBeamBreakSensor){
            loopUntil({shooter.hasNote}){
                groundIntake.passToShooter(shooter)
            }
        }else{
            loopFor(0.5.seconds){
                groundIntake.passToShooter(shooter)
            }
        }
    }

    +aimToLocation(drivetrain, apriltagVision, pivot, FieldLocation.AMP)

    loopFor(0.2.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        shooter.setIdle()
    }

    +pivot.setAngleCommand(PivotAngle.STOWED)
}