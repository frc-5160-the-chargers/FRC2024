package frc.robot

import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.RobotConfig
import frc.external.frc254.CrashTracker

val ROBOT = ChargerRobot(
    getRobotContainer = { RobotContainer() },
    config = RobotConfig(
        isReplay = false,
        tuningMode = false,
        onError = { CrashTracker.logThrowableCrash(it) },
        hardwareConfigRetryLimit = 3
    )
)