@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.framework

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand

/**
 * Represents a basic Robot Container, for use in Command-Based programming.
 */
public abstract class ChargerRobotContainer {

    public abstract val autonomousCommand: Command

    // instantCommand is used over Commands.idle() in case the test command isn't canceled for some reason
    public open val testCommand: Command get() = InstantCommand{}

    /*
    Here are functions that replicate functionality of the Robot class.
     */

    /**
     * Called once when robot starts up.
     */
    public open fun robotInit(){}

    /**
     * Called periodically in the robot.
     */
    public open fun robotPeriodic(){}

    /**
     * Called when the robot is disabled.
     */
    public open fun disabledInit() {}

    /**
     * Called repeatedly when the robot is disabled.
     */
    public open fun disabledPeriodic() {}

    /**
     * Called once when the robot's autonomous mode is enabled.
     */
    public open fun autonomousInit() {}

    /**
     * Called repeatedly in autonomous.
     */
    public open fun autonomousPeriodic() {}

    /**
     * Called once when teleop period starts.
     */
    public open fun teleopInit() {}

    /**
     * Called repeatedly during teleop.
     */
    public open fun teleopPeriodic() {}

    /**
     * Called once when test mode is enabled.
     */
    public open fun testInit() {}

    /**
     * Called periodically when test mode is enabled.
     */
    public open fun testPeriodic() {}

    /**
     * Called once when simulation mode starts.
     */
    public open fun simulationInit() {}

    /**
     * Called repeatedly in sim.
     */
    public open fun simulationPeriodic() {}

}