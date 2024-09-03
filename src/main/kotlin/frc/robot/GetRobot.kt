package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import frc.robot.rigatoni.CompetitionRobot

/**
 * A function that returns the current robot.
 *
 * Switch this out to another class to switch the current robot program.
 */
fun getRobot(): TimedRobot = CompetitionRobot()