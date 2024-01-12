@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.advantagekitextensions

import org.littletonrobotics.junction.LogTable

/**
 * Represents a generic class that can push its data to log,
 * and replicate a version of itself from a log file.
 */
public interface AdvantageKitLoggable<out T> {
    public fun pushToLog(table: LogTable, category: String)

    public fun getFromLog(table: LogTable, category: String): T
}