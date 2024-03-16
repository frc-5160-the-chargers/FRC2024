package frc.robot.commands.aiming

import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.hardware.inputdevices.DriverController
import kotlin.math.abs
import kotlin.math.max

/**
 * Runs note pursuit indefinetly if a target is found; else, drives with joysticks.
 *
 * This command does not end by itself; it is recommended to map this to a whileTrue trigger
 * so that an operator can end the command.
 */
fun pursueNoteElseTeleopDrive(
    drivetrain: EncoderHolonomicDrivetrain,
    noteDetector: ObjectVisionPipeline
): Command = buildCommand(logIndividualCommands = true) {
    fun getNotePursuitSpeed(): Double{
        val swerveOutput = DriverController.swerveOutput
        return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - (noteDetector.bestTarget?.tx ?: 0.0) / 50.0) // scales based off of the vision target error
    }

    // regular drive occurs until suitable target found
    loopUntil({
        val distanceToTarget = noteDetector.robotToTargetDistance(targetHeight = 0.meters)
        distanceToTarget != null && distanceToTarget < ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
    }, drivetrain){
        drivetrain.swerveDrive(DriverController.swerveOutput, !DriverController.shouldDisableFieldRelative)
    }

    // then, note pursuit is run
    +pursueNote(drivetrain, noteDetector, ::getNotePursuitSpeed)

    waitFor(0.5.seconds)
}