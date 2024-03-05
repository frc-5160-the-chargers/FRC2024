package frc.robot

import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.RobotConfig

val ROBOT = ChargerRobot(
    getRobotContainer = { TestBoardRobotContainer() },
    config = RobotConfig(
        isReplay = false,
        tuningMode = false,
        onError = {},
        hardwareConfigRetryLimit = 3
    )
)