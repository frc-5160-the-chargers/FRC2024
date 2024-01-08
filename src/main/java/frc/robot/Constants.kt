package frc.robot

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import frc.chargers.controls.pid.PIDConstants

const val AIM_TO_TARGET_ENABLED = true

const val ODOMETRY_UPDATE_FREQUENCY_HZ = 250.0

const val DRIVER_CONTROLLER_PORT = 0

const val OPERATOR_CONTROLLER_PORT = 1

val LIMELIGHT_LENS_HEIGHT = 0.0.meters

val LIMELIGHT_MOUNT_ANGLE = 0.degrees

val AIM_TO_APRILTAG_PID = PIDConstants(0.2,0.0,0.0)