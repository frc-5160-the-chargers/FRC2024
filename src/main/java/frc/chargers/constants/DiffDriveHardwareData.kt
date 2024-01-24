@file:Suppress("unused")
package frc.chargers.constants

import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.units.inches

/**
 * A class used to hold constants for an [frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain].
 */
data class DiffDriveHardwareData(
    val invertMotors: Boolean = false,
    val gearRatio: Double = DEFAULT_GEAR_RATIO,
    val wheelDiameter: Length,
    val width: Length,
){
    companion object{
        fun andyMark(invertMotors: Boolean = false): DiffDriveHardwareData =
            DiffDriveHardwareData(
                invertMotors,
                10.71,
                6.inches,
                27.inches
            )
    }
}