package frc.robot

import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


internal class RobotContainersTest {
    @BeforeEach
    fun setup(){
        assert(HAL.initialize(500,0))
    }

    @Test
    fun `robot containers should initialize`(){
        val compRobotContainer = CompetitionRobotContainer()
        val defenseSimulatorRobotContainer = DefenseSimulatorRobotContainer()
    }
}