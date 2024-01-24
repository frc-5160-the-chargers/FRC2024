@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.constants

import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants


/**
 * A convenience class for holding control parameters of an [frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain].
 */
public open class DiffDriveControlData(
    public val leftVelocityPID: PIDConstants,
    public val leftFF: AngularMotorFFEquation,
    public val rightVelocityPID: PIDConstants,
    public val rightFF: AngularMotorFFEquation,
    public val robotRotationPID: PIDConstants = PIDConstants(0.4,0,0),
    public val pathAlgorithm: PathAlgorithm = PathAlgorithm.LTV,
    public val pathReplanConfig: ReplanningConfig = ReplanningConfig(),
){
    public data object None: DiffDriveControlData(
        PIDConstants(0,0,0),
        AngularMotorFFEquation(0.0,0.0),
        PIDConstants(0,0,0),
        AngularMotorFFEquation(0.0,0.0)
    )

    public enum class PathAlgorithm{
        LTV, RAMSETE
    }
}
