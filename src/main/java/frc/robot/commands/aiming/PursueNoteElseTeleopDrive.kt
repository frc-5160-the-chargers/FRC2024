package frc.robot.commands.aiming

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
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
    noteDetector: ObjectVisionPipeline,
    acceptableDistanceToNoteMargin: Distance = 2.meters
): Command =
    buildCommand {
        fun getNotePursuitSpeed(visionTarget: VisionTarget.Object): Double{
            val swerveOutput = DriverController.swerveOutput
            return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - visionTarget.tx / 50.0) // scales based off of the vision target error
        }

        // regular drive occurs until suitable target found
        loopUntil({
            val distanceToTarget = noteDetector.robotToTargetDistance(targetHeight = 0.meters)
            distanceToTarget != null && distanceToTarget < acceptableDistanceToNoteMargin
        }, drivetrain){
            drivetrain.swerveDrive(DriverController.swerveOutput)
        }

        // then, note pursuit is run
        +pursueNote(drivetrain, noteDetector, ::getNotePursuitSpeed)

        // after no more note detected, still drive in the same rough direction
        loop{
            drivetrain.swerveDrive(
                -abs(getNotePursuitSpeed(VisionTarget.Object.Dummy)), 0.0, 0.0, fieldRelative = false
            )
        }
    }