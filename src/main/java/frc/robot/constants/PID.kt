package frc.robot.constants

import frc.chargers.controls.pid.PIDConstants

object PID{
    // open loop
    val CAMERA_YAW_TO_OPEN_LOOP_STRAFE = PIDConstants(0.14,0,0.02)

    val CAMERA_YAW_TO_ROTATIONAL_VELOCITY = PIDConstants(0.14,0,0.02)

    val ANGLE_TO_ROTATIONAL_VELOCITY = PIDConstants(3.5,0,0)

    val PATH_TRANSLATION_CONTROL = PIDConstants(0.3,0,0)

    val PATH_ROTATION_CONTROL = PIDConstants(0.3,0,0)
}