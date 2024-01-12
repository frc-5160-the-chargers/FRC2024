@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.constants

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.grams
import com.batterystaple.kmeasure.units.kilo
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds

/*
Note: @PublishedApi makes internal values accessible to inline functions.
This allows the use of these variables in places inline functions need to be used,
without making them public explicitly.
 */

@PublishedApi
internal val DEFAULT_MAX_MODULE_SPEED: Velocity = 4.5.ofUnit(meters / seconds)

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

@PublishedApi
internal val DEFAULT_SWERVE_TURN_INERTIA: MomentOfInertia = 0.025.ofUnit(kilo.grams * (meters * meters) )

@PublishedApi
internal val DEFAULT_SWERVE_DRIVE_INERTIA: MomentOfInertia = 0.004096955.ofUnit(kilo.grams * (meters * meters) )
