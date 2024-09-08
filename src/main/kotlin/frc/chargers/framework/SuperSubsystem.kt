package frc.chargers.framework

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.faultchecking.FaultChecking
import frc.chargers.framework.faultchecking.SubsystemFault
import frc.chargers.framework.faultchecking.SystemStatus
import frc.chargers.wpilibextensions.fpgaTimestamp

/**
 * A convenience abstract class that defines a subsystem that supports logging, tuning,
 * and fault-checking capabilities.
 */
abstract class SuperSubsystem(final override val namespace: String): SubsystemBase(), Loggable, Tunable {
    override fun getName(): String = namespace

    companion object {
        private val allSubsystems = mutableListOf<SuperSubsystem>()

        init {
            ChargerRobot.runPeriodicAtPeriod(1.seconds){
                allSubsystems.forEach { it.publishStatus() }
            }
            ChargerRobot.runPeriodicAtPeriod(0.25.seconds) {
                allSubsystems.forEach { it.updateFaults() }
            }
        }
    }

    init {
        @Suppress("LeakingThis")
        allSubsystems.add(this)
    }

    /**
     * Adds a device for fault checking.
     *
     * Example usage:
     *
     * ```
     * val motor = ChargerSparkMax(6).checkForFaults("ArmMotor")
     */
    protected fun <T: FaultChecking> T.checkForFaults(deviceName: String): T {
        faultSuppliers[deviceName] = this
        return this
    }

    /**
     * Gets the current [SystemStatus].
     */
    protected fun getSystemStatus(): SystemStatus {
        var worstStatus: SystemStatus = SystemStatus.OK
        for (f in this.faults) {
            if (f.sticky || f.timestamp > fpgaTimestamp() - 10.seconds) {
                worstStatus = if (f.isWarning && worstStatus !== SystemStatus.ERROR) {
                    SystemStatus.WARNING
                } else {
                    SystemStatus.ERROR
                }
            }
        }
        return worstStatus
    }

    /**
     * Manually adds a [SubsystemFault].
     */
    protected fun addFault(fault: SubsystemFault) {
        faults.add(fault)
    }

    /**
     * Clears all [SubsystemFault]s.
     */
    protected fun clearFaults(){
        faults.clear()
        publishStatus()
    }

    private val faultSuppliers = mutableMapOf<String, FaultChecking>()
    private val faults = mutableListOf<SubsystemFault>()

    private fun publishStatus() {
        val status = getSystemStatus()
        val faultStrings = faults.map { fault ->
            String.format("[%.2f] %s", fault.timestamp, fault.description)
        }
        log("FaultChecking/Status", status)
        log("FaultChecking/Faults", faultStrings)
        log("FaultChecking/LastFault", faultStrings.lastOrNull() ?: "")
    }

    private fun updateFaults() {
        if (RobotBase.isSimulation()) return
        for ((deviceName, faultSupplier) in faultSuppliers) {
            faults.addAll(faultSupplier.getFaults(deviceName))
        }
    }
}