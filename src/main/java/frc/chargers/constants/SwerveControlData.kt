@file:Suppress( "unused")
package frc.chargers.constants

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision

/**
 * A class that holds Control data for a [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain].
 * This includes PID constants, feedforward, and path re-planning configs.
 */
data class SwerveControlData(
    val azimuthControl: SwerveAzimuthControl,
    val velocityPID: PIDConstants,
    val velocityFF: AngularMotorFFEquation,
    val openLoopDiscretizationRate: Double = 2.0,
    val closedLoopDiscretizationRate: Double = 1.0,
    val robotRotationPID: PIDConstants = PIDConstants(0.3,0,0),
    val robotTranslationPID: PIDConstants = PIDConstants(0.3,0,0),
    val pathReplanConfig: ReplanningConfig = ReplanningConfig()
)


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
