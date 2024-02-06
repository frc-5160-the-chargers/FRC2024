package frc.robot.constants

import frc.chargers.controls.pid.PIDConstants

object PID{
    // open loop
    val CAMERA_YAW_TO_OPEN_LOOP_STRAFE = PIDConstants(0.3,0,0)

    val CAMERA_YAW_TO_CLOSED_LOOP_STRAFE = PIDConstants(0.3,0,0)

    val CAMERA_YAW_TO_ROTATIONAL_VELOCITY = PIDConstants(0.3,0,0)

    val ANGLE_TO_ROTATIONAL_VELOCITY = PIDConstants(0.3,0,0)
}