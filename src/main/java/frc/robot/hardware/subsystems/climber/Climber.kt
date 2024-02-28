package frc.robot.hardware.subsystems.climber

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIO

class Climber(private val io: ClimberIO): SubsystemBase() {
    fun setVoltage(voltage: Voltage){
        io.setVoltages(voltage, voltage)
    }

    fun runUpwards(){
        setVoltage(8.volts)
    }

    fun runDownwards(){
        setVoltage(-8.volts)
    }

    fun setIdle(){
        setVoltage(0.volts)
    }
}