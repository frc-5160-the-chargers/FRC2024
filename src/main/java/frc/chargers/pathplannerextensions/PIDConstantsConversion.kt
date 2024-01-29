@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.pathplannerextensions

import frc.chargers.controls.pid.PIDConstants

/**
 * Converts ChargerLib PID constants to pathplanner PID constants.
 */
public fun PIDConstants.asPathPlannerConstants(): com.pathplanner.lib.util.PIDConstants =
    com.pathplanner.lib.util.PIDConstants(kP, kI, kD)