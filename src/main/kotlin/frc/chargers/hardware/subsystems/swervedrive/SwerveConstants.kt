@file:Suppress("unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.controls.motionprofiling.AngularMotionProfile

private val DEFAULT_MAX_MODULE_SPEED = 4.5.ofUnit(meters / seconds)

data class ModuleType(
    val wheelDiameter: Length,
    val turnMotorInverted: Boolean,
    val turnGearRatio: Double,
    val driveGearRatio: Double
){
    companion object {
        val Mk4iL2 = ModuleType(4.inches, true, 150.0 / 7.0, 6.75)
        val Mk4iL3 = ModuleType(4.inches, true, 150.0 / 7.0, 6.12)
        // we don't have maxswerve so there isn't an option here
    }
}

data class SwerveConstants(
    val trackWidth: Length,
    val wheelBase: Length,
    val moduleType: ModuleType,
    val robotRotationPID: PIDConstants = PIDConstants(2.5,0.0,0.0), // rotation of the entire robot; for pathplanner
    val robotTranslationPID: PIDConstants = PIDConstants(4.0,0.0,0.0), // translation of the entire robot; for pathplanner
    val pathReplanningConfig: ReplanningConfig = ReplanningConfig(),
    val odometryUpdateRate: Time = 0.02.seconds,
    val couplingRatio: Double? = null,
    val driveMotorMaxSpeed: Velocity = DEFAULT_MAX_MODULE_SPEED,

    val azimuthPID: PIDConstants,
    val azimuthPIDTolerance: Angle? = null,
    val azimuthMotionProfile: AngularMotionProfile? = null,
    val azimuthFF: AngularMotorFeedforward = AngularMotorFeedforward(0.0, 0.0, 0.0),

    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFeedforward,
)