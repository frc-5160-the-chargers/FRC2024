@file:Suppress("unused")
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.units.inches
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import frc.chargers.controls.feedforward.AngularMotorFeedforward

@PublishedApi
internal const val DEFAULT_GEAR_RATIO: Double = 1.0

enum class PathAlgorithm {
    LTV, RAMSETE
}

class DifferentialDriveConstants(
    val invertMotors: Boolean = false,
    val gearRatio: Double = DEFAULT_GEAR_RATIO,
    val wheelDiameter: Length,
    val width: Length,
    val velocityPID: PIDConstants = PIDConstants(0.0, 0.0, 0.0),
    val velocityFF: AngularMotorFeedforward = AngularMotorFeedforward(0.0, 0.0, 0.0),
    val pathAlgorithm: PathAlgorithm = PathAlgorithm.LTV,
    val pathReplanningConfig: ReplanningConfig = ReplanningConfig(),
){
    companion object{
        fun andymarkKitbot(
            invertMotors: Boolean,
            velocityPID: PIDConstants = PIDConstants(0.0,0.0,0.0),
            velocityFF: AngularMotorFeedforward = AngularMotorFeedforward(0.0,0.0,0.0),
            pathAlgorithm: PathAlgorithm = PathAlgorithm.LTV,
            pathReplanningConfig: ReplanningConfig = ReplanningConfig(),
        ): DifferentialDriveConstants =
            DifferentialDriveConstants(
                invertMotors,
                10.71,
                6.inches,
                27.inches,
                velocityPID, velocityFF, pathAlgorithm,
                pathReplanningConfig
            )
    }
}