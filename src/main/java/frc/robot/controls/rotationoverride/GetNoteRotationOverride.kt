package frc.robot.controls.rotationoverride

import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.AimToObjectRotationOverride

fun getNoteRotationOverride(notePipeline: ObjectVisionPipeline): AimToObjectRotationOverride =
    AimToObjectRotationOverride(
        notePipeline,
        PIDConstants(0.1,0.0,0.003),
        invert = RobotBase.isReal()
    )