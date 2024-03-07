package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.framework.ChargerRobotContainer

class EmptyRobotContainer: ChargerRobotContainer() {
    override val autonomousCommand: Command
        get() = InstantCommand()
}