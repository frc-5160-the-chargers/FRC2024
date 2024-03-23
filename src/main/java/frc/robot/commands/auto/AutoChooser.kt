package frc.robot.commands.auto

import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.aiming.pursueNote
import frc.robot.commands.auto.components.AmpAutoComponent
import frc.robot.commands.auto.components.AmpAutoTaxiMode
import frc.robot.commands.auto.components.AutoStartingPose
import frc.robot.commands.auto.components.SpeakerAutoComponent
import frc.robot.commands.followPathOptimal
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
    val selected: Command get() = sendableChooser.get() ?: PrintCommand("WARNING: An Auto command was requested, but none were set.")


    private val ampScoreNote2Component = AmpAutoComponent.fromPathPlanner(
        grabPathName = "AmpGrabG1",
        scorePathName = "AmpScoreG1",
        type = AmpAutoComponent.Type.SCORE_NOTE
    )

    private val ampScoreNote3Component = AmpAutoComponent.fromPathPlanner(
        grabPathName = "AmpGrabG3",
        scorePathName = "AmpScoreG3",
        type = AmpAutoComponent.Type.SCORE_NOTE
    )

    /*
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
     */

    val speakerAutoTest = if (aprilTagVision != null && noteDetector != null){
        speakerAutonomous(
            noteDetector, drivetrain,
            shooter, pivot, groundIntake,
            blueStartingPose = AutoStartingPose.SPEAKER_CENTER_BLUE,
            additionalComponents = listOf(
                SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.1", scorePathName = "5pAutoCenter.2",),
                SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.3", scorePathName = "5pAutoCenter.4",),
                SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.5", scorePathName = "5pAutoCenter.6",),
                SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.7", scorePathName = "5pAutoCenter.8",)
            )
        )
    }else{
        InstantCommand()
    }

    init{
        sendableChooser.apply{
            addDefaultOption("Just Taxi", basicTaxi(drivetrain))

            // known to work
            addOption(
                "2 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    additionalComponents = listOf(ampScoreNote2Component)
                )
            )


            addOption(
                "2 Note Amp + Taxi(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    additionalComponents = listOf(ampScoreNote2Component),
                    taxiMode = AmpAutoTaxiMode.TAXI_LONG
                )
            )

            addOption("Do Nothing", InstantCommand())

            addOption("Trolling Speaker Side", followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("Trolling")))

            addOption("Trolling Source Side", followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("Trolling2")))

            addOption(
                "1 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake
                )
            )

            addOption(
                "1 Note Amp + Short Taxi(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
                    taxiMode = AmpAutoTaxiMode.TAXI_SHORT
                )
            )

            addOption(
                "3 Note Amp(NO VISION)",
                noVisionAmpAutonomous(
                    drivetrain, shooter, pivot,
                    groundIntake,
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
                        shooter, pivot, groundIntake
                    )
                )

                addOption(
                    "2 Note Amp",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake,
                        additionalComponents = listOf(ampScoreNote2Component)
                    )
                )

                addOption(
                    "2 Note Amp + Taxi",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiMode = AmpAutoTaxiMode.TAXI_LONG,
                        additionalComponents = listOf(ampScoreNote2Component)
                    )
                )

                /*
                addOption(
                    "2 Note Amp + 1-2 Note Ferry",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake, taxiAtEnd = true,
                        additionalComponents = listOf(ampScoreNote2Component) + ferryComponents
                    )
                )

                 */

                addOption(
                    "3 Note Amp",
                    ampAutonomous(
                        aprilTagVision, noteDetector, drivetrain,
                        shooter, pivot, groundIntake,
                        additionalComponents = listOf(ampScoreNote2Component, ampScoreNote3Component)
                    )
                )



                addOption(
                    "4-5 Piece Speaker",
                    speakerAutonomous(
                        noteDetector, drivetrain,
                        shooter, pivot, groundIntake,
                        blueStartingPose = AutoStartingPose.SPEAKER_CENTER_BLUE,
                        additionalComponents = listOf(
                            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.1", scorePathName = "5pAutoCenter.2",),
                            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.3", scorePathName = "5pAutoCenter.4",),
                            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.5", scorePathName = "5pAutoCenter.6",),
                            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.7", scorePathName = "5pAutoCenter.8",)
                        )
                    )
                )

            }
        }
    }
}
