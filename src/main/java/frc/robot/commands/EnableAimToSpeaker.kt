package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.runOnceCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToAprilTagRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.ANGLE_TO_ROTATIONAL_VELOCITY_PID
import kotlin.jvm.optionals.getOrNull


private val AIM_TO_SPEAKER_VISION_PID = PIDConstants(0.11,0,0.02)


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
            AIM_TO_SPEAKER_VISION_PID,
            aimToTagPoseIfNotFound = true,
            angleToVelocityPID = ANGLE_TO_ROTATIONAL_VELOCITY_PID
        )
    )
}