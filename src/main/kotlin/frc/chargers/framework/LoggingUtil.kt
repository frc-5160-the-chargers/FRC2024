package frc.chargers.framework

import edu.wpi.first.wpilibj.Timer
import monologue.Logged
import monologue.Monologue

/**
 * Logs the latency of a certain function.
 * There is no runtime overhead for logging latency like this.
 *
 * @param key: The namespace of which latency is logged.
 * @param function: The function that will be logged.
 */
inline fun <T> logLatency(key: String, function: () -> T): T {
    val startTime = Timer.getFPGATimestamp()
    val returnValue = function()
    Monologue.log("$key(MS)", (Timer.getFPGATimestamp() - startTime) * 1000)
    return returnValue
}

fun Logged.logNullable(key: String, value: Double?) {
    if (value == null) {
        log("$key/value", 0.0)
        log("$key/present", false)
    } else {
        log("$key/value", value)
        log("$key/present", true)
    }
}
