@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.pathplannerextensions

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.GoalEndState
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d

public fun GoalEndState(
    velocity: Velocity,
    rotation: Angle
): GoalEndState = GoalEndState(
    velocity.inUnit(meters / seconds), rotation.asRotation2d()
)