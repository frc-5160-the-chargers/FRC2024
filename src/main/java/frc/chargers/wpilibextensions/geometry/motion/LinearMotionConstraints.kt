@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.motion

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.trajectory.TrapezoidProfile

/**
 * A holder class that represents the constraints of a mechanism with linear acceleration/velocity,
 * with the respective [maxVelocity] and [maxAcceleration].
 *
 * @see TrapezoidProfile.Constraints
 */
public data class LinearMotionConstraints(
    val maxVelocity: Velocity,
    val maxAcceleration: Acceleration
){
    public constructor(siValue: TrapezoidProfile.Constraints): this(
        Quantity(siValue.maxVelocity),
        Quantity(siValue.maxAcceleration)
    )

    public fun inUnit(distanceUnit: Distance, timeUnit: Time): TrapezoidProfile.Constraints =
        TrapezoidProfile.Constraints(
            maxVelocity.inUnit(distanceUnit / timeUnit),
            maxAcceleration.inUnit(distanceUnit / timeUnit / timeUnit)
        )

    public val siValue: TrapezoidProfile.Constraints =
        inUnit(meters,seconds)
}