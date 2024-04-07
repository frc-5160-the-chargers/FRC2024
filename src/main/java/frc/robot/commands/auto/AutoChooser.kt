package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
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
        groundIntakePreSpinupTime = 0.5.seconds
    )

    private val ampScoreNote3Component = AmpAutoComponent.fromPathPlanner(
        grabPathName = "AmpGrabG2",
        scorePathName = "AmpScoreG2"
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

    val speakerAutoTest = speakerAutonomous(
        drivetrain, shooter, pivot, groundIntake, noteDetector,
        blueStartingPose = AutoStartingPose.SPEAKER_CENTER_BLUE,
        additionalComponents = listOf(
            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.1", scorePathName = "5pAutoCenter.2",),
            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.5", scorePathName = "5pAutoCenter.6",),
            SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.7", scorePathName = "5pAutoCenter.8",)
        )
    )

    val ampAutoTest =
        ampAutonomous(
            drivetrain, shooter, pivot, groundIntake, noteDetector,
            additionalComponents = listOf(ampScoreNote2Component, ampScoreNote3Component)
        )

    init{
        sendableChooser.apply{
            // noteDetector is nullable
            // if null, it will not be used in the following commands(as they accept a nullable ObjectVisionPipeline)
            addDefaultOption("Just Taxi", basicTaxi(drivetrain))

            addOption("Do Nothing", InstantCommand())

            addOption("Trolling Speaker Side", followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("Trolling")))

            addOption("Trolling Source Side", followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("Trolling2")))

            addOption(
                "1 Note Amp",
                ampAutonomous(drivetrain, shooter, pivot, groundIntake, noteDetector)
            )

            addOption(
                "1 Note Amp + Short Taxi",
                ampAutonomous(
                    drivetrain, shooter, pivot, groundIntake, noteDetector,
                    taxiMode = AmpAutoTaxiMode.TAXI_SHORT
                )
            )

            addOption(
                "2.5-3 Note Amp",
                ampAutonomous(
                    drivetrain, shooter, pivot, groundIntake, noteDetector,
                    additionalComponents = listOf(ampScoreNote2Component, ampScoreNote3Component),
                    taxiMode = AmpAutoTaxiMode.NO_TAXI
                )
            )

            addOption(
                "2.5-3 Note Amp(NO VISION; WORKING)",
                ampAutonomous(
                    drivetrain, shooter, pivot, groundIntake, null,
                    additionalComponents = listOf(ampScoreNote2Component, ampScoreNote3Component),
                    taxiMode = AmpAutoTaxiMode.NO_TAXI
                )
            )

            addOption(
                "Untested - One Note Speaker + Taxi(Source Side; No Pathplanner)",
                onePieceSpeakerAndTaxi(drivetrain, shooter, groundIntake, pivot, AutoStartingPose.SPEAKER_RIGHT_BLUE)
            )

            addOption(
                "Untested - Two Piece Speaker(Center; No Pathplanner)",
                twoPieceSpeakerAndTaxi(drivetrain, shooter, groundIntake, pivot)
            )

            addOption(
                "Untested - 4-5 Piece Speaker(Center Side)",
                speakerAutonomous(
                    drivetrain, shooter, pivot, groundIntake, noteDetector,
                    blueStartingPose = AutoStartingPose.SPEAKER_CENTER_BLUE,
                    additionalComponents = listOf(
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.1", scorePathName = "5pAutoCenter.2",),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.3", scorePathName = "5pAutoCenter.4", spinupShooterDuringGrabPath = false),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.5", scorePathName = "5pAutoCenter.6",),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.7", scorePathName = "5pAutoCenter.8",)
                    )
                )
            )

            addOption(
                "Untested - 4 Piece Speaker(Center Side)",
                speakerAutonomous(
                    drivetrain, shooter, pivot, groundIntake, noteDetector,
                    blueStartingPose = AutoStartingPose.SPEAKER_CENTER_BLUE,
                    additionalComponents = listOf(
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.1", scorePathName = "5pAutoCenter.2",),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.5", scorePathName = "5pAutoCenter.6",),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "5pAutoCenter.7", scorePathName = "5pAutoCenter.8",)
                    )
                )
            )

            addOption(
                "Untested - 3 Piece Speaker(Source Side; 1 center note)",
                speakerAutonomous(
                    drivetrain, shooter, pivot, groundIntake, noteDetector,
                    blueStartingPose = AutoStartingPose.SPEAKER_RIGHT_BLUE,
                    additionalComponents = listOf(
                        SpeakerAutoComponent.fromChoreo(grabPathName = "3pAutoRight.1", scorePathName = "3pAutoRight.2"),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "3pAutoRight.3", scorePathName = "3pAutoRight.4", spinupShooterDuringGrabPath = false)
                    )
                )
            )


            addOption(
                "Untested - 2.5 Piece Speaker(Source Side; 2 center notes)",
                speakerAutonomous(
                    drivetrain, shooter, pivot, groundIntake, noteDetector,
                    blueStartingPose = AutoStartingPose.SPEAKER_RIGHT_BLUE,
                    additionalComponents = listOf(
                        SpeakerAutoComponent.fromChoreo(grabPathName = "3pAutoRight.3", scorePathName = "3pAutoRight.4"),
                        SpeakerAutoComponent.fromChoreo(grabPathName = "3pAutoRight.3", scorePathName = null)
                    )
                )
            )
        }
    }
}
