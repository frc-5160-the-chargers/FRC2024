@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.wpilibextensions.kinematics

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.chargers.framework.ChargerRobot


/**
 * A convenience function that creates a [ChassisSpeeds]
 *
 * with kmeasure units instead.
 */
public fun ChassisSpeeds(xVelocity: Velocity, yVelocity: Velocity, rotationalVelocity: AngularVelocity): ChassisSpeeds =
    ChassisSpeeds(
        xVelocity.inUnit(meters/seconds),
        yVelocity.inUnit(meters/seconds),
        rotationalVelocity.inUnit(radians/seconds)
    )


public val ChassisSpeeds.xVelocity: Velocity
    get() = vxMetersPerSecond.ofUnit(meters/seconds)
public val ChassisSpeeds.yVelocity: Velocity
    get() = vyMetersPerSecond.ofUnit(meters/seconds)
public val ChassisSpeeds.rotationSpeed: AngularVelocity
    get() = omegaRadiansPerSecond.ofUnit(radians/seconds)


/**
 * A function used to correct for drift on swerve drive when simultaneously rotating and driving in a singular direction.
 *
 * This function also allows you to customize the rate of correction, in addition to the loop period.
 */
public fun ChassisSpeeds.discretize(dt: Time? = null, driftRate: Double = 1.0): ChassisSpeeds {
    val period = dt ?: ChargerRobot.LOOP_PERIOD
    val desiredDeltaPose = Pose2d(
        vxMetersPerSecond * period.inUnit(seconds),
        vyMetersPerSecond * period.inUnit(seconds),
        Rotation2d.fromRadians(omegaRadiansPerSecond * period.inUnit(seconds) * driftRate)
    )
    val twistForPose: Twist2d = Pose2d().log(desiredDeltaPose)
    return ChassisSpeeds(
        twistForPose.dx / period.inUnit(seconds),
        twistForPose.dy / period.inUnit(seconds),
        this.omegaRadiansPerSecond
    )
}







