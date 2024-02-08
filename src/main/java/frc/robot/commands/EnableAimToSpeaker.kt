package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.runOnceCommand
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToAprilTagRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.PID
import kotlin.jvm.optionals.getOrNull

fun enableAimToSpeaker(
    drivetrain: EncoderHolonomicDrivetrain,
    apriltagVision: AprilTagVisionPipeline
): Command = runOnceCommand(drivetrain){
    drivetrain.setRotationOverride(
        AimToAprilTagRotationOverride(
            if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                4
            }else{
                7
            },
            apriltagVision,
            PID.CAMERA_YAW_TO_ROTATIONAL_VELOCITY,
            aimToTagPoseIfNotFound = true,
            angleToVelocityPID = PID.ANGLE_TO_ROTATIONAL_VELOCITY
        )
    )
}