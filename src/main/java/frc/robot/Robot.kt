package frc.robot

import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.GitData
import frc.chargers.framework.RobotConfig
import frc.robot.BuildConstants.*

val ROBOT = ChargerRobot(
    { RobotContainer() },
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
        onError = { println("An error has occurred. Normally, this will write to the crash tracker disc. ") }
    )
)