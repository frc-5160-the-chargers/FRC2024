package frc.robot.commands.auto.components

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.ampAutonomous
import frc.robot.commands.auto.basicTaxi
import frc.robot.commands.auto.noVisionAmpAutonomous
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

class AutoChooser(
    aprilTagVision: AprilTagVisionPipeline? = null,
    noteDetector: ObjectVisionPipeline? = null,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
){
    private val sendableChooser = LoggedDashboardChooser<Command>("AutoOptions")

    private val secondNoteScoreComponent = AmpAutoScoreComponent.fromPathPlanner(
        grabPathName = "AmpGrabG1",
        scorePathName = "AmpScoreG1",
        type = AmpAutoScoreComponent.Type.SCORE_NOTE
    )

    private val thirdNoteScoreComponent = AmpAutoScoreComponent.fromPathPlanner(
        grabPathName = "AmpGrabG3",
        scorePathName = "AmpScoreG3",
        type = AmpAutoScoreComponent.Type.SCORE_NOTE
    )

    private val ferryComponents = listOf(
        AmpAutoScoreComponent.fromChoreo(
            grabPathName = "FerryPath.1",
            scorePathName = "FerryPath.2",
            type = AmpAutoScoreComponent.Type.FERRY_NOTE
        ),
        AmpAutoScoreComponent.fromChoreo(
            grabPathName = "FerryPath.3",
            scorePathName = "FerryPath.4",
            type = AmpAutoScoreComponent.Type.FERRY_NOTE
        )
    )

    init{
        sendableChooser.apply{
            addOption("Do Nothing", InstantCommand())

            addDefaultOption("Just Taxi", basicTaxi(drivetrain))

            addOption(
                "1 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = false,
                )
            )

            addOption(
                "1 Note Amp + Taxi(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = true
                )
            )

            addOption(
                "2 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = false,
                    additionalComponents = listOf(secondNoteScoreComponent)
                )
            )

            addOption(
                "2 Note Amp + Taxi(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = true,
                    additionalComponents = listOf(secondNoteScoreComponent)
                )
            )

            addOption(
                "3 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = true,
                    additionalComponents = listOf(secondNoteScoreComponent)
                )
            )

            if (aprilTagVision != null && noteDetector != null){
                addOption(
                    "Taxi + Pursue note",
                    basicTaxi(drivetrain).andThen(pursueNote(drivetrain, noteDetector))
                )

                addOption(
                    "1 Note Amp",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = false,
                    )
                )

                addOption(
                    "1 Note Amp + Taxi",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = true,
                    )
                )

                addOption(
                    "2 Note Amp",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = false,
                        additionalComponents = listOf(secondNoteScoreComponent)
                    )
                )

                addOption(
                    "2 Note Amp + Taxi",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = true,
                        additionalComponents = listOf(secondNoteScoreComponent)
                    )
                )

                addOption(
                    "3 Note Amp",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = false,
                        additionalComponents = listOf(secondNoteScoreComponent, thirdNoteScoreComponent)
                    )
                )
            }



            /*
            addOption(
                "1 Note Amp",
                noVisionAmpAutonomous()
                ampAutonomous(
                    aprilTagVision, noteDetector,
                    drivetrain,
                    shooter, pivot, groundIntake
                )
                oneNoteAmp(drivetrain = drivetrain, shooter = shooter, pivot = pivot, stowPivotAtEnd = true)
            )

            addOption(
                "1 Note Amp + Taxi",
                oneNoteAmp(drivetrain = drivetrain, shooter = shooter, pivot = pivot, stowPivotAtEnd = true, taxiAtEnd = true)
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

             */
        }
    }


    val selected: Command
        get() = sendableChooser.get() ?: InstantCommand()
}