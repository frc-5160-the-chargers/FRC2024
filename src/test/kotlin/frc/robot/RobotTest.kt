package frc.robot

import edu.wpi.first.hal.HAL
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
        TestingRobot()
        PushBot()
    }
}