package frc.chargers.framework

import edu.wpi.first.wpilibj.Timer


/**
 * Logs the latency of a certain function.
 * There is no runtime overhead for logging latency like this.
 *
 * @param identifier: The namespace of which latency is logged.
 * @param function: The function that will be logged.
 */
inline fun <T> Loggable.logLatency(identifier: String, function: () -> T): T{
    val startTime = Timer.getFPGATimestamp()
    val returnValue = function()
    log("$identifier(MS)", (Timer.getFPGATimestamp() - startTime) * 1000)
    return returnValue
}