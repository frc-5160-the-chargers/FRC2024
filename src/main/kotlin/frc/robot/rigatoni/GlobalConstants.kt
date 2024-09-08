package frc.robot.rigatoni

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.wpilibextensions.PathConstraints

/* Command/Control Constants */
val ACCEPTABLE_DISTANCE_BEFORE_PATHFIND = 0.3.meters
val ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE = 0.7.meters
val AIM_TO_ANGLE_PID = PIDConstants(1.7,0.0,0.001)
val AIM_TO_NOTE_PID = PIDConstants(0.05, 0.0, 0.001)
// custom overload of path constraints that uses kmeasure quantities
val PATHFIND_CONSTRAINTS = PathConstraints(
    Velocity(4.0),
    Acceleration(4.0),
    500.degrees / 1.seconds,
    650.degrees / 1.seconds / 1.seconds
)

/* Controller Constants */
const val DRIVER_CONTROLLER_PORT = 0
const val OPERATOR_CONTROLLER_PORT = 1

const val DRIVER_RIGHT_HANDED = false
const val DEFAULT_DEADBAND = 0.1

const val SHOOTER_SPEED_MULTIPLIER = -0.8
const val PIVOT_SPEED_MULTIPLIER = -0.33
const val SHOOTER_DEADBAND = 0.15
const val PIVOT_DEADBAND = 0.33