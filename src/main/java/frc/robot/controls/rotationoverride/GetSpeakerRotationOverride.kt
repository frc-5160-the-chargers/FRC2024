package frc.robot.controls.rotationoverride

import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToAprilTagRotationOverride
import frc.robot.ANGLE_TO_ROTATIONAL_VELOCITY_PID
import kotlin.jvm.optionals.getOrNull

fun getSpeakerRotationOverride(aprilTagPipeline: AprilTagVisionPipeline): AimToAprilTagRotationOverride =
    AimToAprilTagRotationOverride(
        if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
            4
        }else{
            7
        },
        aprilTagPipeline,
        PIDConstants(0.13,0.0,0.02),
        aimToTagPoseIfNotFound = true,
        angleToVelocityPID = ANGLE_TO_ROTATIONAL_VELOCITY_PID
    )