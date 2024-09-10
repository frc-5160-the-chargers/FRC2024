@file:Suppress("unused")
package frc.chargers.wpilibextensions

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints

fun GoalEndState(velocity: Velocity, rotation: Angle) = GoalEndState(
    velocity.inUnit(meters / seconds), Rotation2d(rotation)
)

fun PathConstraints(
    maxLinearVelocity: Velocity,
    maxLinearAcceleration: Acceleration,
    maxAngularVelocity: AngularVelocity,
    maxAngularAcceleration: AngularAcceleration
) = PathConstraints(
    maxLinearVelocity.inUnit(meters / seconds),
    maxLinearAcceleration.inUnit(meters / seconds / seconds),
    maxAngularVelocity.inUnit(radians / seconds),
    maxAngularAcceleration.inUnit(radians / seconds / seconds)
)