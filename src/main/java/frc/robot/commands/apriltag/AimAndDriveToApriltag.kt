package frc.robot.commands.apriltag

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.revertIfInvalid
import org.littletonrobotics.junction.Logger

private val aprilTagField = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

fun aimAndDriveToApriltag(
    wantedDistance: Distance,
    targetId: Int,
    distanceTargetPID: PIDConstants,
    aimingPID: PIDConstants,
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: AprilTagVisionPipeline,
): Command {
    val targetHeight: Distance =
        aprilTagField
            .getTagPose(targetId)
            .orElseThrow { Exception("Your apriltag target ID is not valid.") }
            .y
            .ofUnit(meters)

    var previousDistance = visionIO.distanceToTarget(targetHeight) ?: Distance(0.0)

    val distanceTargetController = SuperPIDController(
        distanceTargetPID,
        getInput = {
            visionIO.distanceToTarget(targetHeight)
                .revertIfInvalid(previousDistance)
                .also { previousDistance = it }
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
                .also { Logger.recordOutput("AppliedDriveOutput", it) }
        },
        drivetrain, visionIO
    )
}