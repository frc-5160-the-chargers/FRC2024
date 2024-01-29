package frc.robot.commands

import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID

/**
 * A helper class to store aiming configuration
 * for commands.
 */
@Suppress("unused")
data class AimingConfig<T: VisionTarget>(
    val vision: VisionPipeline<T>,
    val pidConstants: PIDConstants = OPEN_LOOP_TRANSLATION_PID
)
