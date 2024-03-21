package frc.robot

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.pathplannerextensions.PathConstraints
import frc.chargers.wpilibextensions.Alert

/*
This file stores all global constants on the robot
which are used in more than one subsystem/command.

Overall, we follow this pattern:
1. All "constants"/data that is exclusive to 1 subsystem/command of the robot
   is usually a constructor parameter, or a private in-file top level constant.
   (For instance, pivot motion profile, pivot PID constants, drivetrain azimuth PID, etc.)
2. All constants that are required in multiple areas of robot code are stored here.

Found Here:
PID constants/alerts that are used in multiple places within the code
Pathfinding constraints
Odometry Frequency
Alerts

Found elswhere:
PID, feedforward, motion profiles, and other constants that are subsystem-specific(RobotContainer)
or command-specific(within the command file/as a parameter).
Motors, and their respective configuration(RobotContainer)
 */

const val ODOMETRY_UPDATE_FREQUENCY_HZ = 200.0

val PATHFIND_CONSTRAINTS = PathConstraints(
    Velocity(4.0),
    Acceleration(4.0),
    500.degrees / 1.seconds,
    650.degrees / 1.seconds / 1.seconds
)

val ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE = 1.5.meters
val ACCEPTABLE_DISTANCE_BEFORE_NOTE_SPINUP = 1.5.meters

// alerts
val NO_TARGET_FOUND_ALERT = Alert.warning(text = "A command is attempting to aim to a target using vision, but none can be found.")

// pid constants used in multiple places
val ANGLE_TO_ROTATIONAL_VELOCITY_PID = PIDConstants(3.8,0,0)



