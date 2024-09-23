@file:Suppress("unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.controls.motionprofiling.AngularMotionProfile

private val DEFAULT_MAX_MODULE_SPEED = 4.5.ofUnit(meters / seconds)
private val DEFAULT_MAX_MODULE_ROTATION_SPEED = 1080.ofUnit(degrees / seconds)
private val DEFAULT_MAX_MODULE_ACCELERATION = 22.5.ofUnit(meters / seconds / seconds)
private const val DEFAULT_GEAR_RATIO: Double = 1.0

enum class ModuleType(
    val wheelDiameter: Length,
    val turnMotorInverted: Boolean,
    val turnGearRatio: Double,
    val driveGearRatio: Double
){
    MK4iL2(4.inches, true, 150.0 / 7.0, 6.75),
    MK4iL3(4.inches, true, 150.0 / 7.0, 6.12)
    // we don't have maxswerve so there isn't an option here
}

data class SwerveConstants(
    val trackWidth: Length,
    val wheelBase: Length,
    val moduleType: ModuleType,
    val robotRotationPID: PIDConstants = PIDConstants(2.5,0.0,0.0),
    val robotTranslationPID: PIDConstants = PIDConstants(4.0,0.0,0.0),
    val pathReplanningConfig: ReplanningConfig = ReplanningConfig(),
    val openLoopDiscretizationRate: Double = 2.0,
    val closedLoopDiscretizationRate: Double = 1.0,
    val odometryUpdateRate: Time = 0.02.seconds,
    val couplingRatio: Double? = null,

    val turnMotorMaxSpeed: AngularVelocity = DEFAULT_MAX_MODULE_ROTATION_SPEED,
    val driveMotorMaxSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,
    val driveMotorMaxAcceleration: Acceleration = DEFAULT_MAX_MODULE_ACCELERATION,

    val azimuthPID: PIDConstants,
    val azimuthPIDTolerance: Angle? = null,
    val azimuthMotionProfile: AngularMotionProfile? = null,
    val azimuthFF: AngularMotorFeedforward = AngularMotorFeedforward(0.0, 0.0, 0.0),

    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFeedforward,
)