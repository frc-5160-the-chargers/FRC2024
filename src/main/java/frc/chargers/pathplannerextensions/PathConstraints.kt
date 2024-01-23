@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.pathplannerextensions

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.PathConstraints

public fun PathConstraints(
    maxLinearVelocity: Velocity,
    maxLinearAcceleration: Acceleration,
    maxAngularVelocity: AngularVelocity,
    maxAngularAcceleration: AngularAcceleration
): PathConstraints = PathConstraints(
    maxLinearVelocity.inUnit(meters / seconds),
    maxLinearAcceleration.inUnit(meters / seconds / seconds),
    maxAngularVelocity.inUnit(radians / seconds),
    maxAngularAcceleration.inUnit(radians / seconds / seconds)
)