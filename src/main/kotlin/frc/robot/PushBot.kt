package frc.robot

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.chargers.framework.ChargerRobot

class PushBot: ChargerRobot() {
    private val xboxController = CommandXboxController(1)

    private val ledTest = DigitalOutput(4)


    private val timer = Timer()
    private var tick = false

    init{
        addPeriodic(
            {
                ledTest.set(tick)
                tick = !tick
                log("CurrentDIORequest", tick)
            },
            2.0
        )
    }
}