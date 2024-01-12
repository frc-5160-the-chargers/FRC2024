@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.pathplannerextensions

import frc.chargers.controls.pid.PIDConstants

public fun PIDConstants.asPathPlannerConstants(): com.pathplanner.lib.util.PIDConstants =
    com.pathplanner.lib.util.PIDConstants(kP,kI,kD)