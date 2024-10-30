package frc.robot

import frc.chargers.framework.ChargerRobot
import frc.robot.pushbot.PushBot
import frc.robot.rigatoni.CompetitionRobot

/**
 * A function that returns the current robot.
 *
 * Switch this out to another class to switch the current robot program.
 */
fun getRobot(): ChargerRobot = PushBot()