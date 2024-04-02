@file:Suppress("unused")
package frc.robot.hardware.subsystems.led

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class LEDController(
    private val digitalOutput1: DigitalOutput,
    private val digitalOutput2: DigitalOutput
): SubsystemBase() {
    fun setToDefault(){
        digitalOutput1.set(false)
        digitalOutput2.set(false)
        Logger.recordOutput("LEDController/broadcast", "Default")
    }

    fun setToNoteInShooter(){
        digitalOutput1.set(true)
        digitalOutput2.set(false)
        Logger.recordOutput("LEDController/broadcast", "NoteInShooter")
    }

    fun setToNoteInGroundIntake(){
        digitalOutput1.set(false)
        digitalOutput2.set(true)
        Logger.recordOutput("LEDController/broadcast", "NoteInGroundIntake")
    }
}