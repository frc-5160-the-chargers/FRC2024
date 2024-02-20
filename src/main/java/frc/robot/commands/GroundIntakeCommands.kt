@file:Suppress("unused")
package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.shooter.Shooter


private val AIM_TO_NOTE_PID = PIDConstants(0.04, 0.0, 0.003)


/**
 * Paths, then ground intakes to the shooter.
 *
 * @see GroundIntakeSerializer.intakeToShooter
 */
fun driveThenGroundIntakeToShooter(
    path: PathPlannerPath? = null,
    noteDetector: ObjectVisionPipeline,

    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,
    getNotePursuitPower: () -> Double = {0.15},
): Command = driveToNoteAndGrab(
    path, false,
    noteDetector, drivetrain, shooter,
    pivot, groundIntake, getNotePursuitPower
)

/**
 * Paths, then ground intakes to the shooter.
 *
 * @see GroundIntakeSerializer.intakeAndStow
 */
fun driveThenGroundIntakeAndStow(
    path: PathPlannerPath? = null,

    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    getNotePursuitPower: () -> Double = {0.15},
): Command = driveToNoteAndGrab(
    path, true,
    noteDetector, drivetrain, shooter,
    pivot, groundIntake, getNotePursuitPower
)


fun driveToNoteAndGrab(
    path: PathPlannerPath? = null,
    stowNoteInGroundIntake: Boolean,

    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    getNotePursuitPower: () -> Double = {0.15},
): Command = buildCommand(name = "Path & Ground Intake", logIndividualCommands = true){
    addRequirements(drivetrain, groundIntake, pivot)

    runOnce{
        noteDetector.reset()

        drivetrain.setRotationOverride(
            AimToObjectRotationOverride(
                noteDetector,
                AIM_TO_NOTE_PID
            )
        )
    }

    runParallelUntilFirstCommandFinishes{
        // parallel #1
        runSequentially{
            if (path != null){
                +AutoBuilder.followPath(path)
            }

            fun shouldContinueDriving(): Boolean =
                if (stowNoteInGroundIntake || !shooter.hasBeamBreakSensor){
                    noteDetector.visionTargets.isNotEmpty()
                }else{
                    shooter.hasNote
                }

            // fieldRelative = false because rotation override makes drivetrain aim to gamepiece;
            // this means that driving back while field relative is not true will directly grab the gamepiece
            loopWhile(::shouldContinueDriving){
                drivetrain.swerveDrive(-getNotePursuitPower(),0.0,0.0, fieldRelative = false)
            }
        }

        // parallel #2
        loop{
            if (stowNoteInGroundIntake){
                groundIntake.intakeAndStow(pivot)
            }else{
                groundIntake.intakeToShooter(pivot, shooter)
            }
        }
    }

    runOnce{
        drivetrain.stop()
        drivetrain.removeRotationOverride()
        shooter.setIdle()
        pivot.setIdle()
        groundIntake.setIdle()
    }
}