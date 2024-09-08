package frc.chargers.framework.faultchecking

import frc.chargers.wpilibextensions.fpgaTimestamp

data class SubsystemFault(
    val description: String,
    val isWarning: Boolean = false,
    val sticky: Boolean = false
) {
    val timestamp = fpgaTimestamp()
}