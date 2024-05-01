package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.BuildCommandScope
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenRed
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.RotationOverrides
import frc.robot.subsystems.GroundIntakeSerializer
import frc.robot.subsystems.NoteObserver
import frc.robot.subsystems.pivot.Pivot
import frc.robot.subsystems.pivot.PivotAngle
import frc.robot.subsystems.shooter.Shooter

@Suppress("unused")
class AutoCommandManager(
    private val drivetrain: EncoderHolonomicDrivetrain,
    private val shooter: Shooter,
    private val groundIntake: GroundIntakeSerializer,
    private val pivot: Pivot,
    private val noteObserver: NoteObserver
) {
    private val sendableChooser = SendableChooser<Command>().apply{
        setDefaultOption(
            "Basic Taxi",
            buildCommand { 
                addRequirements(drivetrain)
                loopFor(5.seconds){ drivetrain.swerveDrive(0.2, 0.0, 0.0) }
                onEnd{ drivetrain.stop() }
            }
        )

        addOption(
            "Do Nothing",
            buildCommand{ }
        )
    }

    /**
     * Creates an auto command that automatically adds itself to the sendable chooser,
     * and requires all the subsystems necessary.
     *
     * This way, values defined by autoCommand("AutoName"){ block }
     * will automatically be registered to a sendable chooser.
     *
     * @see buildCommand
     */
    private inline fun autoCommand(
        autoName: String,
        builderScope: BuildCommandScope.() -> Unit
    ): Command {
        // autos require all of the subsystems
        val command = buildCommand(name = autoName){
            addRequirements(drivetrain, shooter, groundIntake, pivot, noteObserver)
            builderScope()
        }
        sendableChooser.addOption(autoName, command)
        return command
    }




    private fun driveAndIntake(
        path: PathPlannerPath,
        useGroundIntakeSensor: Boolean = true,
        spinUpShooterDuringPath: Boolean = false,
        groundIntakePreSpinupTime: Time = 0.seconds,
        groundIntakePostSpinupTime: Time = 0.seconds,
    ): Command = buildCommand {
        // starts ground intake a little before path
        loopFor(groundIntakePreSpinupTime){ groundIntake.intake() }

        runParallelUntilFirstCommandFinishes {
            if (useGroundIntakeSensor && noteObserver.hasGroundIntakeSensor){
                // if ground intake sensor is used,
                // have the entire parallel command group stop when the note observer has a note in the serializer.
                // since the waitUntil command is now the first command, it will stop every other command when the condition becomes true.
                waitUntil{ noteObserver.state == NoteObserver.State.NoteInSerializer }
            }

            // parallel #1
            runSequentially {
                +followPathOptimal(drivetrain, path)

                // drives back to grab note
                loopWhile({noteObserver.state is NoteObserver.State.NoteDetected}){
                    drivetrain.swerveDrive(0.3, 0.0, 0.0, fieldRelative = false)
                }

                waitFor(groundIntakePostSpinupTime)
            }

            // parallel #2
            +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

            // parallel #3
            loop{
                groundIntake.intake()
                if (spinUpShooterDuringPath){
                    shooter.outtakeAtSpeakerSpeed()
                }else{
                    shooter.setIdle()
                }
            }

            // parallel #4
            if (noteObserver.hasCamera){
                // rotation override set is delayed as to prevent the drivetrain from aiming to a random note
                // along the path.
                runSequentially{
                    val grabPathStartPose = path.pathPoses.last().ofUnit(meters).flipWhenRed()

                    waitUntil{ drivetrain.robotPose.distanceTo(grabPathStartPose) < 0.8.meters }

                    runOnce{ drivetrain.setRotationOverride(RotationOverrides.aimToNote(noteObserver)) }
                }
            }
        }

        onEnd{
            drivetrain.removeRotationOverride()
        }
    }

    private fun driveAndScoreAmp(
        path: PathPlannerPath,
    ): Command = buildCommand {
        runParallelUntilAllFinish {
            +followPathOptimal(drivetrain, path)

            runSequentially {
                +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                +passSerializedNote(noteObserver, groundIntake, shooter)
            }
        }

        +shootInAmp(noteObserver, shooter, pivot)
    }

    private fun driveAndScoreSpeaker(
        path: PathPlannerPath,
        shooterSpinUpTime: Time = 0.seconds
    ): Command = buildCommand {
        runParallelUntilFirstCommandFinishes{
            +followPathOptimal(drivetrain, path)

            // just run shooting to bring the shooter up to speed
            loop{
                shooter.outtakeAtSpeakerSpeed()
                groundIntake.setIdle()
                pivot.setAngle(PivotAngle.SPEAKER)
            }
        }

        +shootInSpeaker(
            noteObserver, shooter, groundIntake, pivot,
            shooterSpinUpTime
        )
    }

    private fun ampAutoStartup(): Command = buildCommand{
        runParallelUntilAllFinish{
            runSequentially{
                // flipWhenNeeded is an extension function of a UnitPose2d
                // that performs alliance flip.
                runOnce{
                    drivetrain.resetPose(AutoStartingPose.AMP_BLUE.flipWhenRed())
                }
                waitFor(0.3.seconds)
                +followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("DriveToAmp"))
            }

            +pivot.setAngleCommand(PivotAngle.AMP)
        }

        +shootInAmp(noteObserver, shooter, pivot)
    }

    private fun speakerAutoStartup(blueStartingPose: UnitPose2d): Command = buildCommand{
        runOnce{
            drivetrain.resetPose(blueStartingPose.flipWhenRed())
        }

        +shootInSpeaker(noteObserver, shooter, groundIntake, pivot, shooterSpinUpTime = 1.5.seconds)
    }





    val onePieceAmpTaxi = autoCommand("1 Piece Amp + Taxi"){
        +ampAutoStartup()
        +pivot.setAngleCommand(PivotAngle.STOWED)
        +followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("AmpSideTaxiShort"))
    }

    val ampMultiPiece = autoCommand("2-3 Piece Amp"){
        +ampAutoStartup()
        +driveAndIntake(
            PathPlannerPath.fromPathFile("AmpGrabG1"),
            groundIntakePreSpinupTime = 0.5.seconds,
            groundIntakePostSpinupTime = 0.5.seconds
        )
        +driveAndScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG1"))
        +driveAndIntake(
            PathPlannerPath.fromPathFile("AmpGrabG2"),
            groundIntakePostSpinupTime = 0.5.seconds
        )
        +driveAndScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG2"))
    }

    val onePieceSpeakerLeft = autoCommand("1 Piece Speaker"){
        +speakerAutoStartup(AutoStartingPose.SPEAKER_LEFT_BLUE)
    }

    val onePieceSpeakerLeftTaxi = autoCommand("1 Piece Speaker + Taxi"){
        +speakerAutoStartup(AutoStartingPose.SPEAKER_LEFT_BLUE)
        loopFor(5.seconds){
            drivetrain.swerveDrive(0.2,0.0,0.0)
        }
        onEnd{
            drivetrain.stop()
        }
    }

    val multiPieceSpeakerCenter = autoCommand("4-5 Piece Speaker"){
        +speakerAutoStartup(AutoStartingPose.SPEAKER_CENTER_BLUE)

        +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.1"), spinUpShooterDuringPath = true)
        +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.2"))

        +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.5"), spinUpShooterDuringPath = true)
        +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.6"))

        +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.7"), spinUpShooterDuringPath = true)
        +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.8"))

        +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.3"))
        +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.4"))
    }

    /**
     * The chosen auto from the auto chooser.
     */
    val selectedAuto: Command get() = sendableChooser.selected
}