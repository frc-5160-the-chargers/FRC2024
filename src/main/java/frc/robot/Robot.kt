package frc.robot

import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.GitData
import frc.chargers.framework.RobotConfig
import frc.external.frc254.CrashTracker
import frc.robot.BuildConstants.*

val ROBOT = ChargerRobot(
    getRobotContainer = { RobotContainer() },
    gitData = GitData(
        projectName = MAVEN_NAME,
        buildDate = BUILD_DATE,
        sha = GIT_SHA,
        branch = GIT_BRANCH,
        dirty = DIRTY
    ),
    config = RobotConfig(
        isReplay = false,
        tuningMode = false,
        onError = { CrashTracker.logThrowableCrash(it) },
        hardwareConfigRetryLimit = 3
    )
)