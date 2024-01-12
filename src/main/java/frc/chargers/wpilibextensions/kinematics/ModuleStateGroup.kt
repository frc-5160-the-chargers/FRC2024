@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d


/**
 * A helper class that stores [SwerveModuleState]s in a more clear way.
 * This is usually preferred over an array, as it is clear which [SwerveModuleState] corresponds to which module.
 */
public open class ModuleStateGroup(
    public var topLeftSpeed: Velocity = Velocity(0.0),
    public var topRightSpeed: Velocity = Velocity(0.0),
    public var bottomLeftSpeed: Velocity = Velocity(0.0),
    public var bottomRightSpeed: Velocity = Velocity(0.0),

    public var topLeftAngle: Angle = Angle(0.0),
    public var topRightAngle: Angle = Angle(0.0),
    public var bottomLeftAngle: Angle = Angle(0.0),
    public var bottomRightAngle: Angle = Angle(0.0)
) {

    
    public constructor(
        topLeftState: SwerveModuleState,
        topRightState: SwerveModuleState,
        bottomLeftState: SwerveModuleState,
        bottomRightState: SwerveModuleState
    ): this(
        topLeftState.speedMetersPerSecond.ofUnit(meters/seconds),
        topRightState.speedMetersPerSecond.ofUnit(meters/seconds),
        bottomLeftState.speedMetersPerSecond.ofUnit(meters/seconds),
        bottomRightState.speedMetersPerSecond.ofUnit(meters/seconds),
        
        topLeftState.angle.asAngle(),
        topRightState.angle.asAngle(),
        bottomLeftState.angle.asAngle(),
        bottomRightState.angle.asAngle()
    )

    public fun toArray(): Array<SwerveModuleState> = a[topLeftState,topRightState,bottomLeftState,bottomRightState]

    public fun desaturate(maxAttainableSpeed: Velocity){
        // !! operator is 100% safe: list size is greater than 0.
        val maxSpeed = listOf(topLeftSpeed,topRightSpeed,bottomLeftSpeed,bottomRightSpeed).maxOrNull()!!
        if (maxSpeed > maxAttainableSpeed){
            topLeftSpeed = topLeftSpeed / maxSpeed * maxAttainableSpeed
            topRightSpeed = topRightSpeed / maxSpeed * maxAttainableSpeed
            bottomLeftSpeed = bottomLeftSpeed / maxSpeed * maxAttainableSpeed
            bottomRightSpeed = bottomRightSpeed / maxSpeed * maxAttainableSpeed
        }
    }

    public inline fun forEachState(action: (SwerveModuleState) -> Unit){
        action(topLeftState)
        action(topRightState)
        action(bottomLeftState)
        action(bottomRightState)
    }
    
    public val topLeftState: SwerveModuleState
        get() = SwerveModuleState(
            topLeftSpeed.inUnit(meters/seconds),
            topLeftAngle.asRotation2d()
        )

    public val topRightState: SwerveModuleState
        get() = SwerveModuleState(
            topRightSpeed.inUnit(meters/seconds),
            topRightAngle.asRotation2d()
        )

    public val bottomLeftState: SwerveModuleState
        get() = SwerveModuleState(
            bottomLeftSpeed.inUnit(meters/seconds),
            bottomLeftAngle.asRotation2d()
        )

    public val bottomRightState: SwerveModuleState
        get() = SwerveModuleState(
            bottomRightSpeed.inUnit(meters/seconds),
            bottomRightAngle.asRotation2d()
        )
}

