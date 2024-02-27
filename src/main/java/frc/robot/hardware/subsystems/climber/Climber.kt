package frc.robot.hardware.subsystems.climber

import com.batterystaple.kmeasure.quantities.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Climber(private val io: ClimberIO): SubsystemBase() {
    fun setVoltage(voltage: Voltage){
        io.setVoltages(voltage, voltage)
    }
}