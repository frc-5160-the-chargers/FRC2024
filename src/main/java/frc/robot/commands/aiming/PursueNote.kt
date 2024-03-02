package frc.robot.commands.aiming

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.controls.rotationoverride.getNoteRotationOverride
import kotlin.math.abs

/**
 * Causes the drivetrain to pursue a note; making it drive to it
 * until the note is no longer visible(likely indicating that it has been intaken).
 */
fun pursueNote(
    drivetrain: EncoderHolonomicDrivetrain,
    noteDetector: ObjectVisionPipeline,
    getNotePursuitSpeed: (VisionTarget.Object) -> Double = { visionTarget -> 0.6 * (0.5 - visionTarget.tx / 100.0) },
    acceptableDistanceToNoteMargin: Distance = 2.meters // determines the maximum distance that vision targets can be from the robot before being rejected
): Command = buildCommand {
    lateinit var currentTarget: VisionTarget.Object

    fun shouldContinuePursuit(): Boolean {
        val allTargets = noteDetector.visionTargets

        if (allTargets.isEmpty()) return false

        for (target in allTargets){
            // notes are on the ground; thus, no height is factored in
            val distance = noteDetector.robotToTargetDistance(targetHeight = 0.meters, target)
            if (distance != null && distance < acceptableDistanceToNoteMargin){
                currentTarget = target
                return true
            }
        }

        return false
    }


    runOnce{
        drivetrain.setRotationOverride(getNoteRotationOverride(noteDetector))
    }

    // fieldRelative = false because rotation override makes drivetrain aim to gamepiece;
    // this means that driving back while field relative is not true will directly grab the gamepiece
    loopWhile(::shouldContinuePursuit){
        val notePursuitPower = try{
            -abs(getNotePursuitSpeed(currentTarget))
        }catch(e: UninitializedPropertyAccessException){
            0.0
        }
        // no rotation needed because rotation override set
        drivetrain.swerveDrive(notePursuitPower,0.0,0.0, fieldRelative = false)
    }

    runOnce{
        drivetrain.stop()
        drivetrain.removeRotationOverride()
    }
}