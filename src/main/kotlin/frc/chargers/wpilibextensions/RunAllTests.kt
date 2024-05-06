package frc.chargers.wpilibextensions

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

/**
 * Runs all SysId routine tests at once, sequentially.
 */
fun SysIdRoutine.runAllTests(): Command =
    quasistatic(SysIdRoutine.Direction.kForward)
        .andThen(quasistatic(SysIdRoutine.Direction.kReverse))
        .andThen(dynamic(SysIdRoutine.Direction.kForward))
        .andThen(dynamic(SysIdRoutine.Direction.kForward))