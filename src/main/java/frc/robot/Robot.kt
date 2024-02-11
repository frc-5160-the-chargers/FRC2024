package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.RobotConfig
import frc.external.frc254.CrashTracker

val ROBOT = ChargerRobot(
    getRobotContainer = { CompetitionRobotContainer() },
    config = RobotConfig(
        isReplay = false,
        tuningMode = false,
        onError = {
            if (RobotBase.isReal()) {
                CrashTracker.logThrowableCrash(it)
            } else {
                println("An error occurred; however, it was not logged to a crash tracker due to simulation.")
            }
        },
        hardwareConfigRetryLimit = 3
    )
)