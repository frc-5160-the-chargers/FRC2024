package frc.debugging

import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

class EmptyRobot: LoggedRobot() {
    override fun robotInit() {
        setUseTiming(false)
        Logger.addDataReceiver(NT4Publisher())
        Logger.start()
    }
}