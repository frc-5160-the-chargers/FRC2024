package frc.robot.commands.aiming

import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import kotlin.math.abs

@PublishedApi
internal val DISTANCE_TO_TARGET_MARGIN = 2.meters

/**
 * Causes the drivetrain to pursue a note; making it drive to it
 * until the note is no longer visible(likely indicating that it has been intaken).
 */
inline fun pursueNote(
    drivetrain: EncoderHolonomicDrivetrain,
    noteDetector: ObjectVisionPipeline,
    crossinline getNotePursuitSpeed: () -> Double = {
        val bestTarget = noteDetector.bestTarget
        if (bestTarget != null){
            0.6 * (0.5 - bestTarget.tx / 100.0)
        }else{
            0.3
        }
    },
    crossinline endCondition: () -> Boolean = { noteDetector.bestTarget == null },
    setRotationOverride: Boolean = true
): Command = buildCommand {
    if (setRotationOverride){
        runOnce{
            drivetrain.setRotationOverride(getNoteRotationOverride(noteDetector))
        }
    }


    loopUntil({
        val targetDistance = noteDetector.robotToTargetDistance(targetHeight = 0.meters)
        endCondition() || (targetDistance != null && targetDistance < DISTANCE_TO_TARGET_MARGIN)
    }){
        // no rotation needed because rotation override set
        drivetrain.swerveDrive(-abs(getNotePursuitSpeed()),0.0,0.0, fieldRelative = false)
    }

    onEnd{
        drivetrain.stop()
        drivetrain.removeRotationOverride()
    }
}