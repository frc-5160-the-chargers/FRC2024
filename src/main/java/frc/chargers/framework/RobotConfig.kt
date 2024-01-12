@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds

/**
 * A generic data class that configures a [ChargerRobot].
 */
public data class RobotConfig(
    val tuningMode: Boolean,
    val isReplay: Boolean,
    val hardwareConfigRetryLimit: Int = 3,
    val extraLoggerConfig: () -> Unit = {},
    val onError: (Throwable) -> Unit = {},
    val loopPeriod: Time = 0.02.seconds,
    val logToNetworkTables: Boolean = true
)