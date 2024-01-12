@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.constants

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.units.inches
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants

/**
 * A class used to hold constants for an [frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain].
 */
public data class DiffDriveHardwareData(
    val invertMotors: Boolean = false,
    val gearRatio: Double = DEFAULT_GEAR_RATIO,
    val wheelDiameter: Length,
    val width: Distance,
){
    public companion object{
        public fun andyMark(invertMotors: Boolean = false): DiffDriveHardwareData =
            DiffDriveHardwareData(
                invertMotors,
                10.71,
                6.inches,
                27.inches
            )
    }
}

/**
 * A convenience class for holding control parameters of an [frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain].
 */
public open class DiffDriveControlData(
    public val leftVelocityPID: PIDConstants,
    public val leftFF: AngularMotorFFConstants,
    public val rightVelocityPID: PIDConstants,
    public val rightFF: AngularMotorFFConstants,
    public val robotRotationPID: PIDConstants = PIDConstants(0.4,0.0,0.0),
    public val pathAlgorithm: PathAlgorithm = PathAlgorithm.LTV,
    public val pathReplanConfig: ReplanningConfig = ReplanningConfig(),
){
    public data object None: DiffDriveControlData(
        PIDConstants(0.0,0.0,0.0),
        AngularMotorFFConstants.None,
        PIDConstants(0.0,0.0,0.0),
        AngularMotorFFConstants.None
    )

    public enum class PathAlgorithm{
        LTV, RAMSETE
    }

}
