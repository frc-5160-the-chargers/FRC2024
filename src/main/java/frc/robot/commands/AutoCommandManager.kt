package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.commandbuilder.BuildCommandScope
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.robot.RotationOverrides
import frc.robot.subsystems.GroundIntakeSerializer
import frc.robot.subsystems.NoteObserver
import frc.robot.subsystems.pivot.Pivot
import frc.robot.subsystems.pivot.PivotAngle
import frc.robot.subsystems.shooter.Shooter

class AutoCommandManager(
    private val drivetrain: EncoderHolonomicDrivetrain,
    private val shooter: Shooter,
    private val groundIntake: GroundIntakeSerializer,
    private val pivot: Pivot,
    private val noteObserver: NoteObserver
) {
    private val sendableChooser = SendableChooser<Command>().apply{
        setDefaultOption(
            "BasicTaxi",
            buildCommand { 
                addRequirements(drivetrain)
                loopFor(5.seconds){ drivetrain.swerveDrive(-0.2, 0.0, 0.0) }
                onEnd{ drivetrain.stop() }
            }
        )
    }

    /**
     * Creates an auto command that automatically adds itself to the sendable chooser,
     * and requires all the subsystems necessary.
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

    /**
     * The chosen auto from the auto chooser.
     */
    val selectedAuto: Command get() = sendableChooser.selected



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
                    val grabPathStartPose = path.pathPoses.last().ofUnit(meters).flipWhenNeeded()

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

        if (RobotBase.isReal()){
            // outtake should not take more than 2 seconds
            runSequentiallyFor(2.seconds){
                loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){
                    shooter.outtakeAtAmpSpeed()
                }
                loopUntil({noteObserver.state == NoteObserver.State.NoNote}){
                    shooter.outtakeAtAmpSpeed()
                }
                loopFor(0.3.seconds, shooter){
                    shooter.outtakeAtAmpSpeed()
                }
            }
        }else{
            loopFor(0.7.seconds, shooter){
                shooter.outtakeAtAmpSpeed()
            }
        }
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


    private val grabSecondAmpNote = driveAndIntake(
        PathPlannerPath.fromPathFile("AmpGrabG1"),
        groundIntakePreSpinupTime = 0.5.seconds,
        groundIntakePostSpinupTime = 0.5.seconds
    )
    private val scoreSecondAmpNote = driveAndScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG1"))

    private val grabThirdAmpNote = driveAndIntake(
        PathPlannerPath.fromPathFile("AmpGrabG2"),
        groundIntakePostSpinupTime = 0.5.seconds
    )
    private val scoreThirdAmpNote = driveAndScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG2"))


    val onePieceAmpTaxi = autoCommand("One Piece Amp + Taxi"){
    }


    
}