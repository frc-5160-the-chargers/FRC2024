@file:Suppress("unused")
package frc.chargers.constants

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.inches
import edu.wpi.first.math.system.plant.DCMotor

/**
 * A class that holds Hardware constants for a [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain].
 * This includes Track width, wheelbase, inertia, turn motor inversion, and max speed.
 */
data class SwerveHardwareData(
    val turnMotorType: DCMotor,
    val driveMotorType: DCMotor,
    val invertTurnMotors: Boolean = false,
    // offsets the encoder readings to theoretically improve odometry accuracy
    val couplingRatio: Double? = null,
    val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    val turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
    val driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
    val maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val maxModuleAcceleration: Acceleration = DEFAULT_MAX_MODULE_SPEED / Time(1.0) * 5.0,
    val maxModuleRotationSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
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
            turnMotorType: DCMotor,
            driveMotorType: DCMotor,
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            maxModuleAcceleration: Acceleration = DEFAULT_MAX_MODULE_SPEED / Time(1.0) * 5.0,
            maxModuleRotationSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
            useCouplingRatio: Boolean = false,
            trackWidth: Length,
            wheelBase: Length,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            turnMotorType,
            driveMotorType,
            invertTurnMotors = true,
            if (useCouplingRatio) 50.0 / 14.0 else null,
            150.0 / 7.0,
            6.75,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            maxModuleAcceleration,
            maxModuleRotationSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L3 swerve modules.
         */
        fun mk4iL3(
            turnMotorType: DCMotor,
            driveMotorType: DCMotor,
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            maxModuleAcceleration: Acceleration = DEFAULT_MAX_MODULE_SPEED / Time(1.0) * 5.0,
            maxModuleRotationSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
            useCouplingRatio: Boolean = false,
            trackWidth: Length,
            wheelBase: Length,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
        ): SwerveHardwareData = SwerveHardwareData(
            turnMotorType,
            driveMotorType,
            invertTurnMotors = true,
            if (useCouplingRatio) 50.0 / 14.0 else null,
            150.0 / 7.0,
            6.12,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            maxModuleAcceleration,
            maxModuleRotationSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )

        /**
         * Creates a [SwerveHardwareData] instance with auto-completed constants
         * related to MK4i L1 swerve modules.
         */
        fun mk4iL1(
            turnMotorType: DCMotor,
            driveMotorType: DCMotor,
            maxModuleSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            useCouplingRatio: Boolean = false,
            maxModuleAcceleration: Acceleration = DEFAULT_MAX_MODULE_SPEED / Time(1.0) * 5.0,
            maxModuleRotationSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
            turnInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_TURN_INERTIA,
            driveInertiaMoment: MomentOfInertia = DEFAULT_SWERVE_DRIVE_INERTIA,
            trackWidth: Length,
            wheelBase: Length
        ): SwerveHardwareData = SwerveHardwareData(
            turnMotorType,
            driveMotorType,
            invertTurnMotors = true,
            if (useCouplingRatio) 50.0 / 14.0 else null,
            150.0 / 7.0,
            8.14,
            turnInertiaMoment,
            driveInertiaMoment,
            maxModuleSpeed,
            maxModuleAcceleration,
            maxModuleRotationSpeed,
            4.inches,
            trackWidth,
            wheelBase
        )
    }

}