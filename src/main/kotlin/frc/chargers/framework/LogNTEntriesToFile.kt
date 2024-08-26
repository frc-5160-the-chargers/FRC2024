package frc.chargers.framework

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DataLogManager

/**
 * Logs all NT entries with a specific prefix
 * to DataLog files.
 *
 * @param entryPrefix: All Network table entries with this prefix will be logged to file.
 * @param logPrefix: All data logged entries will be under this category within the datalog file.
 */
@Suppress("unused")
fun logNTEntriesToFile(entryPrefix: String, logPrefix: String = "NT:"){
    NetworkTableInstance.getDefault().startEntryDataLog(
        DataLogManager.getLog(), entryPrefix, logPrefix
    )
}