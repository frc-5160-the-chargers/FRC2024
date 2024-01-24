@file:Suppress("unused")
package frc.chargers.constants

import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants


/**
 * A convenience class for holding control parameters of an [frc.chargers.hardware.subsystems.differentialdrive.EncoderDifferentialDrivetrain].
 */
open class DiffDriveControlData(
    val leftVelocityPID: PIDConstants,
    val leftFF: AngularMotorFFEquation,
    val rightVelocityPID: PIDConstants,
    val rightFF: AngularMotorFFEquation,
    val robotRotationPID: PIDConstants = PIDConstants(0.4,0,0),
    val pathAlgorithm: PathAlgorithm = PathAlgorithm.LTV,
    val pathReplanConfig: ReplanningConfig = ReplanningConfig(),
){
    data object None: DiffDriveControlData(
        PIDConstants(0,0,0),
        AngularMotorFFEquation(0.0,0.0),
        PIDConstants(0,0,0),
        AngularMotorFFEquation(0.0,0.0)
    )

    enum class PathAlgorithm{
        LTV, RAMSETE
    }
}
