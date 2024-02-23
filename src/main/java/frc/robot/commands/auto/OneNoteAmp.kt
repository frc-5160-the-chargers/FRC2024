package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.commands.FieldLocation
import frc.robot.commands.aimToLocation
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import kotlin.jvm.optionals.getOrNull

fun oneNoteAmp(
    apriltagVision: AprilTagVisionPipeline? = null,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,

    stowPivotAtEnd: Boolean = false,
    taxiAtEnd: Boolean = false
): Command = buildCommand(name = "One Note Amp"){
    addRequirements(drivetrain, shooter, pivot)

    runOnce{
        drivetrain.poseEstimator.resetPose(
            UnitPose2d(1.4.meters, 7.3.meters, 90.degrees).flipWhenNeeded()
        )
    }

    if (apriltagVision != null){
        val flipAlliance by getOnceDuringRun{
            DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red
        }

        loopFor(0.2.seconds){
            if (flipAlliance){
                drivetrain.swerveDrive(0.13, 0.13, 0.0, fieldRelative = false)
            }else{
                drivetrain.swerveDrive(0.13, -0.13, 0.0, fieldRelative = false)
            }
            pivot.setVoltage(2.volts)
            println("Hi!!!!!!!!")
        }

        +aimToLocation(
            drivetrain,
            apriltagVision,
            pivot,
            FieldLocation.AMP
        )
    }else{
        runParallelUntilAllFinish{
            +AutoBuilder.followPath(
                PathPlannerPath.fromPathFile("DriveToAmp")
            )

            +pivot.setAngleCommand(PivotAngle.AMP)
        }
    }

    loopFor(0.2.seconds){
        shooter.outtake(0.5)
    }

    runOnce{
        shooter.setIdle()
    }

    runParallelUntilAllFinish{
        if (taxiAtEnd){
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))
        }

        if (stowPivotAtEnd){
            +pivot.setAngleCommand(PivotAngle.STOWED)
        }
    }
}

