package frc.robot

import edu.wpi.first.hal.HAL
import frc.robot.pushbot.PushBot
import frc.robot.rigatoni.CompetitionRobot
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


internal class RobotTest {
    @BeforeEach
    fun setup(){
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `robot classes should initialize`(){
        CompetitionRobot()
        PushBot()
    }
}