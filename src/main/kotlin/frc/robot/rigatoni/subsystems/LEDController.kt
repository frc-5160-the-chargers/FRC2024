package frc.robot.rigatoni.subsystems

import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDController: SubsystemBase() {
    val serialPort = SerialPort(9600, SerialPort.Port.kUSB)
    val timer = Timer()

    var tick = false

    override fun periodic(){
        if (timer.advanceIfElapsed(2.0)){
            tick = !tick
            serialPort.write(byteArrayOf(if(tick) 2 else 5), 0)
        }
    }
}