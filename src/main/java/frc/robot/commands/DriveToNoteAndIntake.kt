@file:Suppress("unused")
package frc.robot.commands

import com.batterystaple.kmeasure.units.meters
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot


private val AIM_TO_NOTE_PID = PIDConstants(0.03, 0.0, 0.003)


fun driveToNoteAndIntake(
    noteDetector: ObjectVisionPipeline,
    drivetrain: EncoderHolonomicDrivetrain,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,


    path: PathPlannerPath? = null,
    getNotePursuitPower: (() -> Double)? = {0.15},
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


            if (getNotePursuitPower != null){
                fun shouldContinuePursuit(): Boolean {
                    val allTargets = noteDetector.visionTargets

                    if (allTargets.isEmpty()) return false

                    val acceptableDistanceToNote =
                        if (DriverStation.isAutonomous()) 0.5.meters else 3.meters

                    for (target in allTargets){
                        // notes are on the ground; thus, no height is factored in
                        val distance = noteDetector.robotToTargetDistance(targetHeight = 0.meters, target)
                        if (distance != null && distance < acceptableDistanceToNote){
                            println("Acceptable distance: $distance")
                            return true
                        }else{
                            println("Target too far: $distance")
                        }
                    }

                    return false
                }

                // fieldRelative = false because rotation override makes drivetrain aim to gamepiece;
                // this means that driving back while field relative is not true will directly grab the gamepiece
                loopWhile(::shouldContinuePursuit){
                    drivetrain.swerveDrive(-getNotePursuitPower(),0.0,0.0, fieldRelative = false)
                }
            }
        }

        // parallel #2
        loop{
            // sets pivot angle by itself.
            groundIntake.intake(pivot)
        }
    }

    runOnce{
        println("Stopped!")
        drivetrain.stop()
        drivetrain.removeRotationOverride()
        pivot.setIdle()
        groundIntake.setIdle()
    }
}