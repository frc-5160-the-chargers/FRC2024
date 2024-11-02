package frc.robot

import frc.chargers.framework.UnitTesting
import frc.robot.pushbot.PushBot
import frc.robot.rigatoni.CompetitionRobot
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


internal class RobotTest {
    @BeforeEach
    fun setup() = UnitTesting.setup()

    @Test
    fun `robot classes should initialize`(){
        CompetitionRobot()
        PushBot()
    }

    @AfterEach
    fun cleanup() = UnitTesting.cleanup()
}