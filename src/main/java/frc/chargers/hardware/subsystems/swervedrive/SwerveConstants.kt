@file:Suppress("unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision


private val DEFAULT_MAX_MODULE_SPEED: Velocity = 4.5.ofUnit(meters / seconds)

private val DEFAULT_MAX_MODULE_ROTATION_SPEED: AngularVelocity = 1080.ofUnit(degrees / seconds)

private const val DEFAULT_GEAR_RATIO: Double = 1.0


data class SwerveChassisConstants(
    val trackWidth: Length,
    val wheelBase: Length,
    val robotRotationPID: PIDConstants = PIDConstants(0.3,0,0),
    val robotTranslationPID: PIDConstants = PIDConstants(0.3,0,0),
    val pathReplanningConfig: ReplanningConfig = ReplanningConfig(),
    val openLoopDiscretizationRate: Double = 2.0,
    val closedLoopDiscretizationRate: Double = 1.0
)

data class SwerveModuleConstants(
    val wheelDiameter: Length,
    val useOnboardPID: Boolean = false,

    val turnMotorInverted: Boolean = false,
    val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    val turnMotorMaxSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
    val turnMotorControlScheme: SwerveAzimuthControl,

    val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    val driveMotorMaxSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val driveMotorMaxAcceleration: Acceleration = (driveMotorMaxSpeed.siValue * 5.0).ofUnit(meters / seconds / seconds),
    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFFEquation,

    val couplingRatio: Double? = null,
){
    companion object{
        fun mk4iL2(
            useOnboardPID: Boolean = false,

            turnMotorMaxSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
            turnMotorControlScheme: SwerveAzimuthControl,

            driveMotorMaxSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            driveMotorMaxAcceleration: Acceleration = (driveMotorMaxSpeed.siValue * 5.0).ofUnit(meters / seconds / seconds),
            velocityPID: PIDConstants,
            velocityFF: AngularMotorFFEquation,
        ): SwerveModuleConstants = SwerveModuleConstants(
            4.inches, useOnboardPID, turnMotorInverted = true,
            150.0 / 7.0, turnMotorMaxSpeed, turnMotorControlScheme,

            6.75, driveMotorMaxSpeed,
            driveMotorMaxAcceleration, velocityPID, velocityFF
        )

        fun mk4iL3(
            useOnboardPID: Boolean,

            turnMotorMaxSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
            turnMotorControlScheme: SwerveAzimuthControl,

            driveMotorMaxSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
            driveMotorMaxAcceleration: Acceleration = (driveMotorMaxSpeed.siValue * 5.0).ofUnit(meters / seconds / seconds),
            velocityPID: PIDConstants,
            velocityFF: AngularMotorFFEquation,
        ): SwerveModuleConstants = SwerveModuleConstants(
            4.inches, useOnboardPID, turnMotorInverted = true,
            150.0 / 7.0, turnMotorMaxSpeed, turnMotorControlScheme,

            6.12, driveMotorMaxSpeed,
            driveMotorMaxAcceleration, velocityPID, velocityFF
        )
    }
}

sealed class SwerveAzimuthControl(
    val pidConstants: PIDConstants,
    val precision: Precision<AngleDimension> = Precision.AllowOvershoot
){
    class PID(
        pidConstants: PIDConstants,
        precision: Precision<AngleDimension> = Precision.AllowOvershoot
    ): SwerveAzimuthControl(pidConstants, precision)

    class ProfiledPID(
        pidConstants: PIDConstants,
        precision: Precision<AngleDimension> = Precision.AllowOvershoot,
        val motionProfile: AngularMotionProfile,
        val ffEquation: AngularMotorFFEquation = AngularMotorFFEquation(0.0,0.0)
    ): SwerveAzimuthControl(pidConstants, precision)
}