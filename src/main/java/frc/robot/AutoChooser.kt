package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.ampAutonomous
import frc.robot.commands.auto.basicTaxi
import frc.robot.commands.auto.components.AmpAutoScoreComponent
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

    /**
     * The selected command of our auto chooser.
     */
    val selected: Command get() = sendableChooser.get() ?: runOnceCommand{ println("WARNING: An Auto command was requested, but none were set.") }


    private val ampScoreNote2Component = AmpAutoScoreComponent.fromPathPlanner(
        grabPathName = "AmpGrabG1",
        scorePathName = "AmpScoreG1",
        type = AmpAutoScoreComponent.Type.SCORE_NOTE
    )

    private val ampScoreNote3Component = AmpAutoScoreComponent.fromPathPlanner(
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
                    additionalComponents = listOf(ampScoreNote2Component)
                )
            )

            addOption(
                "2 Note Amp + Taxi(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = true,
                    additionalComponents = listOf(ampScoreNote2Component)
                )
            )

            addOption(
                "2 Note Amp + 1-2 Ferry(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = true,
                    additionalComponents = listOf(ampScoreNote2Component) + ferryComponents
                )
            )

            addOption(
                "3 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiAtEnd = true,
                    additionalComponents = listOf(ampScoreNote2Component)
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
                        additionalComponents = listOf(ampScoreNote2Component)
                    )
                )

                addOption(
                    "2 Note Amp + Taxi",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = true,
                        additionalComponents = listOf(ampScoreNote2Component)
                    )
                )

                addOption(
                    "2 Note Amp + 1-2 Note Ferry",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = true,
                        additionalComponents = listOf(ampScoreNote2Component) + ferryComponents
                    )
                )

                addOption(
                    "3 Note Amp",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = false,
                        additionalComponents = listOf(ampScoreNote2Component, ampScoreNote3Component)
                    )
                )

                /*
                addOption(
                    "4-5 Piece Speaker",
                    speakerAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake,
                        startingPose = SpeakerAutoStartingPose.CENTER,
                        additionalComponents = listOf(
                            SpeakerAutoScoreComponent.fromChoreo(grabPathName = "5pAutoCenter.1", scorePathName = "5pAutoCenter.2", shooterShouldStartDuringPath = true,),
                            SpeakerAutoScoreComponent.fromChoreo(grabPathName = "5pAutoCenter.3", scorePathName = "5pAutoCenter.4", shooterShouldStartDuringPath = true,),
                            SpeakerAutoScoreComponent.fromChoreo(grabPathName = "5pAutoCenter.5", scorePathName = "5pAutoCenter.6", shooterShouldStartDuringPath = true,),
                            SpeakerAutoScoreComponent.fromChoreo(grabPathName = "5pAutoCenter.7", scorePathName = "5pAutoCenter.8", shooterShouldStartDuringPath = true,)
                        )
                    )
                )
                 */

            }
        }
    }
}
