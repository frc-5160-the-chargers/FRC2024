package frc.robot

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.pathplannerextensions.PathConstraints
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.Alert

/*
This file stores all global constants on the robot
which are used in more than one subsystem/command.

Overall, we follow this pattern:
1. All "constants"/data that is exclusive to a subsystem of the robot
   is usually a constructor parameter, and is specified in the RobotContainer.
   (For instance, pivot motion profile, pivot PID constants, drivetrain azimuth PID, etc.)
2. All constants that are required in multiple areas of robot code are stored here.

Found Here:
Command aiming PID and precision(camera yaw to drivetrain strafe, for instance);
as these have to stay consistent between different command instances
Pathfinding constraints
Odometry Frequency
Alerts

Found within the RobotContainer:
PID, feedforward, motion profiles, and other constants that are subsystem-specific
Motors, and their respective configuration.
 */

const val ODOMETRY_UPDATE_FREQUENCY_HZ = 200.0

val PATHFIND_CONSTRAINTS = PathConstraints(
    Velocity(4.0),
    Acceleration(6.0),
    500.degrees / 1.seconds,
    650.degrees / 1.seconds / 1.seconds
)

// alerts
val NO_TARGET_FOUND_ALERT = Alert.warning(text = "A command is attempting to aim to an apriltag, but none can be found.")

// aiming pid constants for commands
val CAMERA_YAW_TO_OPEN_LOOP_STRAFE_PID = PIDConstants(0.0115, 0.0,0.004)

val OPEN_LOOP_STRAFE_PRECISION = Precision.Within(Scalar(0.5))

val CAMERA_YAW_TO_ROTATIONAL_VELOCITY_PID = PIDConstants(0.11,0,0.02)

val ANGLE_TO_ROTATIONAL_VELOCITY_PID = PIDConstants(3.5,0,0)

const val DISTANCE_TO_TAG_REACH_KP = 0.5

