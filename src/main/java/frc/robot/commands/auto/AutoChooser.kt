package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object AutoChooser {
    private val sendableChooser = LoggedDashboardChooser<Command>("Autonomous")

    fun initOptions(
        aprilTagVision: AprilTagVisionPipeline? = null,
        noteDetector: ObjectVisionPipeline? = null,

        drivetrain: EncoderHolonomicDrivetrain,
        shooter: Shooter,
        pivot: Pivot,
        groundIntake: GroundIntake,
    ){
        sendableChooser.apply{
            addDefaultOption("Taxi", basicTaxi(drivetrain))

            addOption("Do Nothing", InstantCommand{})
            addOption("Pathplanner Taxi", pathplannerTaxi(drivetrain))
            addOption(
                "Two piece amp(no vision)",
                twoPieceAmpNoVision(drivetrain, shooter, pivot, groundIntake)
            )

            if (aprilTagVision != null && noteDetector != null){
                addOption(
                    "Two piece amp(with vision)",
                    twoPieceAmpWithVision(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake
                    )
                )

                addOption(
                    "Five piece speaker(beta)",
                    fivePieceSpeakerBeta(
                        aprilTagVision, noteDetector , drivetrain,
                        shooter, pivot, groundIntake,
                    )
                )

                addOption(
                    "Six piece speaker(beta)",
                    sixPieceSpeakerBeta(
                        aprilTagVision, noteDetector , drivetrain,
                        shooter, pivot, groundIntake,
                    )
                )
            }
        }
    }

    val selected: Command
        get() = sendableChooser.get()
}