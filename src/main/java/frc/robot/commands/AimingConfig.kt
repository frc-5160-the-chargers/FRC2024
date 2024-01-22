package frc.robot.commands

import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget

val DEFAULT_AIMING_PID = PIDConstants(0.2,0.0,0.0)

/**
 * A helper class to store aiming configuration
 * for commands.
 */
@Suppress("unused")
data class AimingConfig<T: VisionTarget>(
    val vision: VisionPipeline<T>,
    val pidConstants: PIDConstants = DEFAULT_AIMING_PID
)
