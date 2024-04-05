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

    init{
        listOf(digitalOutput1, digitalOutput2, digitalOutput3).forEach{
            //it.enablePWM(0.5)
        }
    }

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

    override fun periodic(){
        Logger.recordOutput("LEDController/digitalOutput1", digitalOutput1.get())
        Logger.recordOutput("LEDController/digitalOutput2", digitalOutput2.get())
        Logger.recordOutput("LEDController/digitalOutput3", digitalOutput3.get())
    }
}