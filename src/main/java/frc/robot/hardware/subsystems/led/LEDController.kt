@file:Suppress("unused")
package frc.robot.hardware.subsystems.led

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class LEDController(
    private val digitalOutput1: DigitalOutput,
    private val digitalOutput2: DigitalOutput,
    private val digitalOutput3: DigitalOutput
): SubsystemBase() {
    fun displayDefault(){
        digitalOutput1.set(false)
        digitalOutput2.set(false)
        digitalOutput3.set(false)
        Logger.recordOutput("LEDController/broadcast", "Default")
    }

    fun displayNoteInShooter(){
        digitalOutput1.set(true)
        digitalOutput2.set(true)
        digitalOutput3.set(true)
        Logger.recordOutput("LEDController/broadcast", "NoteInShooter")
    }

    fun displayNoteInGroundIntake(){
        digitalOutput1.set(true)
        digitalOutput2.set(true)
        digitalOutput2.set(false)
        Logger.recordOutput("LEDController/broadcast", "NoteInGroundIntake")
    }

    fun displayError(){
        digitalOutput1.set(false)
        digitalOutput2.set(false)
        digitalOutput2.set(true)
        Logger.recordOutput("LEDController/broadcast", "Error")
    }

    fun displayShooterSpinup(){
        digitalOutput1.set(false)
        digitalOutput2.set(true)
        digitalOutput2.set(false)
        Logger.recordOutput("LEDController/broadcast", "ShooterSpinup")
    }

    fun displayShooterReady(){
        digitalOutput1.set(true)
        digitalOutput2.set(false)
        digitalOutput2.set(true)
        Logger.recordOutput("LEDController/broadcast", "ShooterReady")
    }
}