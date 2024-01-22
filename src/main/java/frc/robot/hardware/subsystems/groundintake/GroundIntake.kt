package frc.robot.hardware.subsystems.groundintake

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GroundIntake: SubsystemBase() {

    fun spin(voltage: Voltage){
        println("Not yet implemented: voltage set is $voltage")
    }

    fun spin(power: Double){
        spin(power * 12.volts)
    }


}