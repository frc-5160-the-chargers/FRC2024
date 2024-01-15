
@file:Suppress("unused")

package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.revertIfInvalid
import frc.chargers.wpilibextensions.Alert
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.abs

fun aimToApriltag(
    targetId: Int? = null,
    aimingPID: PIDConstants,
    /**
     * A lambda that fetches the forward power that the drivetrain will drive for
     * while aiming PID is happening.
     */
    getDrivePower: () -> Double = {0.0},
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: VisionPipeline<VisionTarget.AprilTag>,
): Command =
    buildCommand(name = "Aim to Apriltag", logIndividualCommands = true){
        val aimingController by getOnceDuringRun {
            SuperPIDController(
                aimingPID,
                getInput = { Scalar(visionIO.bestTarget?.tx ?: 0.0) },
                target = Scalar(0.0),
                outputRange = Scalar(-0.5)..Scalar(0.5)
            )
        }

        fun canFindTarget(): Boolean {
            val bestTarget = visionIO.bestTarget

            return if (bestTarget == null){
                Alert.warning(text =
                "A command is attempting to aim to an apriltag, " +
                        "but none can be found."
                ).active = true
                false
            }else if (targetId != bestTarget.id ){
                Alert.warning(text =
                "An apriltag command can find an apriltag, " +
                        "but it does not match the id of $targetId. " +
                        "(Found id: " + bestTarget.id
                ).active = true
                false
            }else{
                true
            }
        }

        fun hasFinishedAiming(): Boolean {
            val aimingError = aimingController.error.siValue
            recordOutput("AimToAngle/aimingError", aimingError)
            // abs() function overload for Kmeasure Quantities(visionIO.distanceToTarget)
            return abs(aimingError) < 0.05
        }

        addRequirements(drivetrain)

        runOnce{
            visionIO.require()
        }

        loopWhile({ canFindTarget() && !hasFinishedAiming() }){
            drivetrain.swerveDrive(
                xPower = getDrivePower(),
                yPower = aimingController.calculateOutput().siValue,
                rotationPower = 0.0
            )
        }

        runOnce{
            visionIO.removeRequirement()
            drivetrain.stop()
        }
    }


private val aprilTagField = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

fun aimAndDriveToApriltag(
    wantedDistance: Distance,
    targetId: Int,
    distanceTargetPID: PIDConstants,
    aimingPID: PIDConstants,
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: VisionPipeline<VisionTarget.AprilTag>,
): Command{
    val targetHeight: Distance =
        aprilTagField
            .getTagPose(targetId)
            .orElseThrow{ Exception("Your apriltag target ID is not valid.") }
            .y
            .ofUnit(meters)

    var previousDistance = visionIO.distanceToTarget(targetHeight) ?: Distance(0.0)

    val distanceTargetController = SuperPIDController(
        distanceTargetPID,
        getInput = {
            visionIO.distanceToTarget(targetHeight)
                .revertIfInvalid(previousDistance)
                .also{ previousDistance = it }
        },
        target = wantedDistance,
        outputRange = Scalar(-0.5)..Scalar(0.5),
        selfSustain = true
    )

    return aimToApriltag(
        targetId,
        aimingPID,
        {
            distanceTargetController
                .calculateOutput()
                .siValue
                .unaryMinus() // negated because if the measured distance is > distance setpoint, effort should be positive, not negative
                .coerceIn(0.0..Double.POSITIVE_INFINITY) // ensures value is > 0.0
                .also{ recordOutput("AppliedDriveOutput", it) }
        },
        drivetrain, visionIO
    )
}

