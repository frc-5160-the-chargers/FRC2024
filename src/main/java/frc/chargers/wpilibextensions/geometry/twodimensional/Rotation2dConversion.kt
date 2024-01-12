@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.radians as kmeasureRadians
import edu.wpi.first.math.geometry.Rotation2d

/**
 * Instead of using a UnitRotation2d, Chargerlib uses Kmeasure's Angle class instead.
 * WPILib's Rotation2d is essentially a typesafe units system for angles,
 * and the Angle class can do everything Rotation2d does, aside from interpolation(which is added here).
 */


/**
 * Converts an [Angle] to a [Rotation2d].
 */
public fun Angle.asRotation2d(): Rotation2d = Rotation2d.fromRadians(inUnit(kmeasureRadians))

/**
 * Converts a [Rotation2d] to an [Angle].
 */
public fun Rotation2d.asAngle(): Angle = this.radians.ofUnit(kmeasureRadians)

/**
 * Interpolates an [Angle].
 */
public fun Angle.interpolate(endValue: Angle, t: Double): Angle =
    this.asRotation2d().interpolate(endValue.asRotation2d(),t).asAngle()
