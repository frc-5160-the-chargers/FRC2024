package frc.robot.commands.auto.components

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.auto.*
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object AutoChooser {
    private val sendableChooser = LoggedDashboardChooser<Command>("AutoOptions")

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

            addOption(
                "1 Note Amp",
                oneNoteAmp(shooter, pivot).andThen(pivot.setAngleCommand(PivotAngle.STOWED))
            )

            addOption(
                "1 Note Amp + Taxi",
                oneNoteAmp(noteDetector = null, drivetrain, shooter, pivot, groundIntake, AmpAutoEndAction.STOW_PIVOT)
            )

            addOption(
                "2 Note Amp(without vision)",
                twoNoteAmpWithoutVision(drivetrain, shooter, pivot, groundIntake, AmpAutoEndAction.STOW_PIVOT)
            )

            addOption(
                "2 Note Amp(without vision) + Taxi",
                twoNoteAmpWithoutVision(drivetrain, shooter, pivot, groundIntake, AmpAutoEndAction.TAXI)
            )

            addOption(
                "2 Note Amp(without vision) + 2 Note Ferry",
                twoNoteAmpWithoutVision(drivetrain, shooter, pivot, groundIntake, AmpAutoEndAction.FERRY)
            )

            if (aprilTagVision != null && noteDetector != null){
                addOption(
                    "1 Note Amp + Three Note Ferry",
                    oneNoteAmp(noteDetector, drivetrain, shooter, pivot, groundIntake, AmpAutoEndAction.FERRY)
                )

                addOption(
                    "Two Note Amp",
                    twoNoteAmp(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, AmpAutoEndAction.STOW_PIVOT
                    )
                )

                addOption(
                    "Two Note Amp + Taxi",
                    twoNoteAmp(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, AmpAutoEndAction.TAXI
                    )
                )

                addOption(
                    "Two Note Amp + Two Note Ferry",
                    twoNoteAmp(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, AmpAutoEndAction.FERRY
                    )
                )

                /*
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
                 */
            }
        }
    }


    val selected: Command
        get() = sendableChooser.get() ?: InstantCommand{}
}