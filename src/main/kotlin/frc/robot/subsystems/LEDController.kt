package frc.robot.subsystems

import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import frc.chargers.framework.SuperSubsystem

class LEDController: SuperSubsystem("LEDController"){
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