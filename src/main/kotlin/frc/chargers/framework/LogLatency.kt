package frc.chargers.framework

import edu.wpi.first.wpilibj.Timer


/**
 * Logs the latency of a certain function.
 */
inline fun <T> Loggable.logLatency(identifier: String, function: () -> T): T{
    val startTime = Timer.getFPGATimestamp()
    val returnValue = function()
    log("$identifier(MS)", (Timer.getFPGATimestamp() - startTime) * 1000)
    return returnValue
}