@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIO

// standard: + = outtake, - = intake; regardless of voltage set
class Shooter(private val io: ShooterIO): SubsystemBase() {
    val hasBeamBreakSensor: Boolean get() = io.hasBeamBreakSensor

    val hasNote: Boolean get() = io.hasNote

    fun setIdle(){
        io.setIntakeVoltage(0.volts)
    }

    fun intake(percentOut: Double){
        intake(percentOut * 12.volts)
    }

    fun intake(voltage: Voltage){
        if (io.hasNote && io.hasBeamBreakSensor){
            io.setIntakeVoltage(0.volts)
        }else{
            io.setIntakeVoltage(voltage)
        }
    }

    fun outtake(percentOut: Double){
        outtake(percentOut * 12.volts)
    }

    fun outtake(voltage: Voltage){
        require(voltage >= 0.volts){ "Applied voltage must be > 0.volts. To intake, call intake(voltage) or intake(speed) instead." }
        NoteVisualizer.update(voltage)
        io.setIntakeVoltage(voltage)
    }




    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
        NoteVisualizer.update(io.intakeVoltages[0])
    }
}
