package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.RobotConfig
import frc.external.frc254.CrashTracker

val ROBOT = ChargerRobot(
    getRobotContainer = { RobotContainer() },
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
        logFilePath = "C:\\Users\\Daniel_Chen\\Downloads\\Log_24-01-20_15-47-16.wpilog",
        hardwareConfigRetryLimit = 3
    )
)