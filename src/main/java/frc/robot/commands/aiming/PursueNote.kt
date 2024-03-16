package frc.robot.commands.aiming

import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import kotlin.math.abs


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
    forwards: Boolean = true,
    setRotationOverride: Boolean = true
): Command = buildCommand(name = "Pursue Note Command") {
    if (setRotationOverride){
        runOnce{
            drivetrain.setRotationOverride(getNoteRotationOverride(noteDetector))
        }
    }


    val multiplier = if (forwards) 1.0 else -1.0

    loopUntil({
        val targetDistance = noteDetector.robotToTargetDistance(targetHeight = 0.meters)
        endCondition() || (targetDistance != null && targetDistance > ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE)
    }){
        // no rotation needed because rotation override set
        drivetrain.swerveDrive(multiplier * abs(getNotePursuitSpeed()),0.0,0.0, fieldRelative = false)
    }

    onEnd{
        drivetrain.stop()
        drivetrain.removeRotationOverride()
    }
}