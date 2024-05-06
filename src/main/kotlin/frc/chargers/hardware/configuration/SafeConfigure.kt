@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.configuration

/**
 * A utility function to retry configuration application
 * for a set number of times,
 * to ensure that configurations get applied properly.
 *
 * If a [HardwareConfigurable] has known issues of configuration not going through,
 * then use this function.
 *
 * @param configure The function block that configures the motor. If this response returns true,
 * the configuration was successful
 */
public inline fun safeConfigure(
    deviceName: String,
    getErrorInfo: () -> String = {"[Nothing]"},
    configure: () -> Boolean
){
    repeat(3){ index: Int ->
        val iteration = index + 1
        if (configure()){
            println("$deviceName has been successfully configured on attempt #$iteration.")
            return
        }else{
            println("$deviceName did not configure successfully! Retry attempt #$iteration.")
        }
    }
    val errorInfo = getErrorInfo()
    error("CONFIGURATION ERROR: $deviceName failed to configure! " +
            "Configuration was attempted" + 3 + " times. \n" +
            "Additional Info: $errorInfo"
    )
}