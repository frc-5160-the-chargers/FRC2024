@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIO

// standard: + = outtake, - = intake; regardless of voltage set
class Shooter(val io: ShooterIO): SubsystemBase() {
    val hasNoteDetector: Boolean get() = io.hasNoteDetector

    val hasNote: Boolean get() = io.hasNote

    fun setIdle(){
        io.setIntakeVoltage(0.volts)
    }

    fun shootInAmp(){
        outtake(4.volts)
    }

    fun shootInSpeaker(visualize: Boolean = RobotBase.isSimulation()){
        outtake(11.volts)
        if (visualize && !NoteVisualizer.isShootingInSpeaker){
            NoteVisualizer.shootInSpeakerCommand().schedule()
        }
    }

    fun receiveFromSource(){
        intake(-8.volts)
    }

    fun receiveFromGroundIntake(){
        intake(-7.volts)
    }

    fun intake(percentOut: Double){
        intake(percentOut * 12.volts)
    }

    fun intake(voltage: Voltage){
        if (io.hasNoteDetector && io.hasNote){
            io.setIntakeVoltage(0.volts)
        }else{
            io.setIntakeVoltage(voltage)
            if (voltage < -2.volts){
                NoteVisualizer.setHasNote(true)
            }
        }
    }

    fun outtake(percentOut: Double){
        outtake(percentOut * 12.volts)
        NoteVisualizer.setHasNote(false)
    }

    fun outtake(voltage: Voltage){
        require(voltage >= 0.volts){ "Applied voltage must be > 0.volts. To intake, call intake(voltage) or intake(speed) instead." }
        NoteVisualizer.setHasNote(false)
        io.setIntakeVoltage(voltage)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}
