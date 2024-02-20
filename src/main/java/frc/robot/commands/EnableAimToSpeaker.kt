package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.runOnceCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToAprilTagRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ANGLE_TO_ROTATIONAL_VELOCITY_PID
import frc.robot.CAMERA_YAW_TO_ROTATIONAL_VELOCITY_PID
import kotlin.jvm.optionals.getOrNull

fun enableAimToSpeaker(
    drivetrain: EncoderHolonomicDrivetrain,
    apriltagVision: AprilTagVisionPipeline
): Command = runOnceCommand{
    apriltagVision.reset()

    drivetrain.setRotationOverride(
        AimToAprilTagRotationOverride(
            if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                4
            }else{
                7
            },
            apriltagVision,
            CAMERA_YAW_TO_ROTATIONAL_VELOCITY_PID,
            aimToTagPoseIfNotFound = true,
            angleToVelocityPID = ANGLE_TO_ROTATIONAL_VELOCITY_PID
        )
    )
}