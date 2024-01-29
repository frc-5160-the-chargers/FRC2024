@file:Suppress("unused")
package frc.chargers.constants

import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.MomentOfInertia
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.units.inches

/**
 * A class that holds Hardware constants for a [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain].
 * This includes Track width, wheelbase, inertia, turn motor inversion, and max speed.
 */
data class SwerveHardwareData(
    val invertTurnMotors: Boolean = false,
    // offsets the encoder readings to theoretically improve odometry accuracy
    val couplingRatio: Double? = null,
    val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    val turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
    val driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val wheelDiameter: Length,
    val trackWidth: Length,
    val wheelBase: Length
){
    companion object{
        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L2 swerve modules.
         */
        fun mk4iL2(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            useCouplingRatio: Boolean = true,
            trackWidth: Length,
            wheelBase: Length,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            if (useCouplingRatio) 50.0 / 14.0 else null,
            150.0 / 7.0,
            6.75,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L3 swerve modules.
         */
        fun mk4iL3(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            useCouplingRatio: Boolean = true,
            trackWidth: Length,
            wheelBase: Length,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            if (useCouplingRatio) 50.0 / 14.0 else null,
            150.0 / 7.0,
            6.12,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L1 swerve modules.
         */
        fun mk4iL1(
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            useCouplingRatio: Boolean = true,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
            trackWidth: Length,
            wheelBase: Length
        ): SwerveHardwareData = SwerveHardwareData(
            invertTurnMotors = true,
            if (useCouplingRatio) 50.0 / 14.0 else null,
            150.0 / 7.0,
            8.14,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )
    }

}