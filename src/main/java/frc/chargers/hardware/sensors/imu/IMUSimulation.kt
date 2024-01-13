@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.Angle
import edu.wpi.first.math.kinematics.ChassisSpeeds

public object IMUSimulation{
    private var simHeadingGetter: () -> Angle = { Angle(0.0) }
    private var simChassisSpeedsGetter: () -> ChassisSpeeds = { ChassisSpeeds() }

    /**
     * Configures simulation for all IMUs.
     */
    fun configure(
        headingSupplier: () -> Angle = { Angle(0.0) },
        chassisSpeedsSupplier: () -> ChassisSpeeds = { ChassisSpeeds() },
    ){
        simChassisSpeedsGetter = chassisSpeedsSupplier
        simHeadingGetter = headingSupplier
    }

    fun getHeading(): Angle = simHeadingGetter()

    fun getChassisSpeeds(): ChassisSpeeds = simChassisSpeedsGetter()

}
