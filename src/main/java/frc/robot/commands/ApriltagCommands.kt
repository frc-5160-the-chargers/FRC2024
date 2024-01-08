@file:Suppress("unused")

package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.abs
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.abs

fun aimToApriltag(
    targetId: Int? = null,
    pidConstants: PIDConstants,
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: VisionPipeline<VisionTarget.AprilTag>,
): Command =
    buildCommand(name = "Aim to Apriltag", logIndividualCommands = true){
        val aimToTargetPID by getOnceDuringRun {
            SuperPIDController(
                pidConstants,
                getInput = { Scalar(visionIO.bestTarget?.tx ?: 0.0) },
                target = Scalar(0.0),
                outputRange = Scalar(-0.5)..Scalar(0.5)
            )
        }

        fun aimInProgress(): Boolean{
            val currentError = aimToTargetPID.error.siValue
            recordOutput("AimToAngle/currentError", currentError)

            if (targetId != null){
                val bestTarget = visionIO.bestTarget
                if (bestTarget != null && bestTarget.id != targetId){
                    println("Target with the target ID could not be found. Wanted ID: $targetId; Found Id: ${bestTarget.id}")
                    return true
                }
            }

            return abs(currentError) < 0.05
        }

        addRequirements(drivetrain)

        runOnce{
            visionIO.require()
        }

        loopWhile(::aimInProgress){
            drivetrain.swerveDrive(
                xPower = 0.0,
                yPower = aimToTargetPID.calculateOutput().siValue,
                rotationPower = 0.0
            )
        }

        runOnce{
            visionIO.removeRequirement()
        }
    }

fun aimAndDriveToApriltag(
    wantedDistance: Distance,
    targetHeight: Distance,
    drivePower: Double = 0.2,
    targetId: Int? = null,
    pidConstants: PIDConstants,
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: VisionPipeline<VisionTarget.AprilTag>,
): Command =
    buildCommand(name = "Aim to Apriltag And Drive", logIndividualCommands = true){
        val aimToTargetPID by getOnceDuringRun {
            SuperPIDController(
                pidConstants,
                getInput = { Scalar(visionIO.bestTarget?.tx ?: 0.0) },
                target = Scalar(0.0),
                outputRange = Scalar(-0.5)..Scalar(0.5)
            )
        }

        fun aimInProgress(): Boolean{
            val currentError = aimToTargetPID.error.siValue
            recordOutput("AimToAngle/currentError", currentError)

            if (targetId != null){
                val bestTarget = visionIO.bestTarget
                if (bestTarget != null && bestTarget.id != targetId){
                    println("Target with the target ID could not be found. Wanted ID: $targetId; Found Id: ${bestTarget.id}")
                    return true
                }
            }

            // abs() function overload for Kmeasure Quantities(visionIO.distanceToTarget)
            return abs(currentError) < 0.05 && abs(visionIO.distanceToTarget(targetHeight)) <= abs(wantedDistance)
        }

        addRequirements(drivetrain)

        runOnce{
            visionIO.require()
        }

        loopWhile(::aimInProgress){
            drivetrain.swerveDrive(
                xPower = drivePower,
                yPower = aimToTargetPID.calculateOutput().siValue,
                rotationPower = 0.0
            )
        }

        runOnce{
            visionIO.removeRequirement()
        }
    }