package frc.robot

import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.RobotConfig

class Robot: ChargerRobot(
    getRobotContainer = { CompetitionRobotContainer() },
    config = RobotConfig(
        replayModeActive = false,
        tuningMode = false,
        hardwareConfigRetryLimit = 3,
        logFilePath = null
    )
)