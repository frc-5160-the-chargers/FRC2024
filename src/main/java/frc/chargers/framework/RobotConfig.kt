@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds

/**
 * A generic data class that configures a [ChargerRobot].
 */
public data class RobotConfig(
    val tuningMode: Boolean,
    val replayModeActive: Boolean,

    val logFilePath: String? = null,
    val replayFilePath: String? = null,

    val hardwareConfigRetryLimit: Int = 3,
    val extraLoggerConfig: () -> Unit = {},
    val defaultExceptionHandler: (Throwable) -> Unit = {},
    val loopPeriod: Time = 0.02.seconds,
)
