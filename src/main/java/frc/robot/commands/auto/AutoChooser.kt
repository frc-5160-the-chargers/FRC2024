package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
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
        groundIntake: GroundIntakeSerializer,
    ){
        sendableChooser.apply{
            addDefaultOption("Taxi", basicTaxi(drivetrain))

            addOption("Do Nothing", InstantCommand{})
            addOption("Pathplanner Taxi", pathplannerTaxi(drivetrain))
            addOption(
                "Two piece amp(no vision)",
                twoNoteAmpNoVision(drivetrain, shooter, pivot, groundIntake)
            )

            if (aprilTagVision != null && noteDetector != null){
                addOption(
                    "Two piece amp(with vision)",
                    twoNoteAmp(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake
                    )
                )

                addOption(
                    "Five piece speaker(beta)",
                    fiveNoteSpeakerRight(
                        aprilTagVision, noteDetector , drivetrain,
                        shooter, pivot, groundIntake,
                    )
                )

                addOption(
                    "Six piece speaker(beta)",
                    sixNoteSpeakerCenter(
                        aprilTagVision, noteDetector , drivetrain,
                        shooter, pivot, groundIntake,
                    )
                )
            }
        }
    }

    val selected: Command
        get() = sendableChooser.get() ?: InstantCommand{}
}