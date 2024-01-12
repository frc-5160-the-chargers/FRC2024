@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.kinematics.SwerveModulePosition
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d

/**
 * A helper class that stores [SwerveModulePosition]s in a more clear way.
 * This is usually preferred over an array, as it is clear which [SwerveModulePosition] corresponds to which module.
 */
public data class ModulePositionGroup(
    var topLeftDistance: Distance = Distance(0.0),
    var topRightDistance: Distance = Distance(0.0),
    var bottomLeftDistance: Distance = Distance(0.0),
    var bottomRightDistance: Distance = Distance(0.0),

    var topLeftAngle: Angle = Angle(0.0),
    var topRightAngle: Angle = Angle(0.0),
    var bottomLeftAngle: Angle = Angle(0.0),
    var bottomRightAngle: Angle = Angle(0.0)
) {
    public constructor(
        topLeftPosition: SwerveModulePosition,
        topRightPosition: SwerveModulePosition,
        bottomLeftPosition: SwerveModulePosition,
        bottomRightPosition: SwerveModulePosition,
    ): this(
        topLeftPosition.distanceMeters.ofUnit(meters),
        topRightPosition.distanceMeters.ofUnit(meters),
        bottomLeftPosition.distanceMeters.ofUnit(meters),
        bottomRightPosition.distanceMeters.ofUnit(meters),

        // asAngle converts a Rotation2d to the Kmeasure angle class
        topLeftPosition.angle.asAngle(),
        topRightPosition.angle.asAngle(),
        bottomLeftPosition.angle.asAngle(),
        bottomRightPosition.angle.asAngle()
    )

    public fun toArray(): Array<SwerveModulePosition> = a[topLeftPosition,topRightPosition,bottomLeftPosition,bottomRightPosition]

    public inline fun forEachPosition(action: (SwerveModulePosition) -> Unit){
        action(topLeftPosition)
        action(topRightPosition)
        action(bottomLeftPosition)
        action(bottomRightPosition)
    }

    public val topLeftPosition: SwerveModulePosition get() = SwerveModulePosition(
        topLeftDistance.inUnit(meters),
        topLeftAngle.asRotation2d()
    )

    public val topRightPosition: SwerveModulePosition get() = SwerveModulePosition(
        topRightDistance.inUnit(meters),
        topRightAngle.asRotation2d()
    )

    public val bottomLeftPosition: SwerveModulePosition get() = SwerveModulePosition(
        bottomLeftDistance.inUnit(meters),
        bottomLeftAngle.asRotation2d()
    )

    public val bottomRightPosition: SwerveModulePosition get() = SwerveModulePosition(
        bottomRightDistance.inUnit(meters),
        bottomRightAngle.asRotation2d()
    )
}