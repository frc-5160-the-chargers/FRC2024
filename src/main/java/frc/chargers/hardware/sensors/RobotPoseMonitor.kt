@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

/**
 * Represents a pose monitor which combines wheel odometry(and optionally vision data)
 * to produce a robot pose.
 *
 * This class should not be implemented by vision cameras or sensors, as their pose results
 * are not always accurate, and thus the nullable parameters are necessary.
 */
public interface RobotPoseMonitor: HeadingProvider{
    public val robotPose: UnitPose2d

    public fun resetPose(pose: UnitPose2d)

    public fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier)


    public fun zeroPose(): Unit = resetPose(UnitPose2d())

    override val heading: Angle get() = robotPose.rotation
}

