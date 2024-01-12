@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Angle
import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * Configures simulation for all IMUs.
 */
public fun configureIMUSimulation(
    headingSupplier: () -> Angle = { Angle(0.0) },
    chassisSpeedsSupplier: () -> ChassisSpeeds = { ChassisSpeeds() },
){
    getSimChassisSpeeds = chassisSpeedsSupplier
    getSimHeading = headingSupplier
}

internal var getSimChassisSpeeds: () -> ChassisSpeeds = { ChassisSpeeds() }
internal var getSimHeading: () -> Angle = { Angle(0.0) }