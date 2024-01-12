@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.threedimensional

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.geometry.Rotation3d


/**
 * Since Rotation3d is not unit-agnostic(its roll+pitch+yaw constructor requires radians as the unit),
 * there is no UnitRotation3d class.
 */

/**
 * Constructs a [Rotation3d] using the kmeasure [Angle] unit for roll, pitch and yaw.
 */
public fun Rotation3d(roll: Angle, pitch: Angle, yaw: Angle): Rotation3d = Rotation3d(
    roll.inUnit(radians),
    pitch.inUnit(radians),
    yaw.inUnit(radians)
)

public val Rotation3d.xAngle: Angle
    get() = x.ofUnit(radians)
public val Rotation3d.yAngle: Angle
    get() = y.ofUnit(radians)
public val Rotation3d.zAngle: Angle
    get() = z.ofUnit(radians)



