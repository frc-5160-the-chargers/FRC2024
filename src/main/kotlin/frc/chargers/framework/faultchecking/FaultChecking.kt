package frc.chargers.framework.faultchecking

/**
 * Represents a device that can fetch faults.
 */
interface FaultChecking {
    fun getFaults(deviceName: String): List<SubsystemFault>
}