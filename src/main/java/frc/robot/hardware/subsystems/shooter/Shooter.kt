@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.wpilibextensions.interpolation.InterpolatingQuantityTreeMap
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIO
import org.littletonrobotics.junction.Logger.recordOutput

// standard: + = outtake, - = intake; regardless of voltage set
class Shooter(private val io: ShooterIO): SubsystemBase() {

    private val shooterVoltageTreeMap = InterpolatingQuantityTreeMap<DistanceDimension, VoltageDimension>()

    val hasNoteDetector: Boolean get() = io.hasNoteDetector

    val hasNote: Boolean get() = io.hasNote

    fun setIdle(){
        io.setIntakeVoltage(0.volts)
    }

    fun outtakeAtAmpSpeed(){
        outtake(4.volts)
    }

    fun outtakeAtSpeakerSpeed() {
        outtake(12.volts)
    }

    fun receiveFromSource(){
        intake(-8.volts)
    }

    fun receiveFromGroundIntake(){
        intake(5.volts)
    }

    fun intake(percentOut: Double){
        intake(percentOut * 12.volts)
    }

    fun intake(voltage: Voltage){
        if (io.hasNoteDetector && io.hasNote){
            io.setIntakeVoltage(0.volts)
        }else{
            io.setIntakeVoltage(voltage)
            if (abs(voltage) > 2.volts){
                NoteVisualizer.setHasNote(true)
            }
        }
        recordOutput("Shooter/isOuttaking", false)
        recordOutput("Shooter/isIntaking", true)
    }

    fun outtake(percentOut: Double){
        outtake(percentOut * 12.volts)
        NoteVisualizer.setHasNote(false)
    }

    fun outtake(voltage: Voltage){
        require(voltage >= 0.volts){ "Applied voltage must be > 0.volts. To intake, call intake(voltage) or intake(speed) instead." }
        NoteVisualizer.setHasNote(false)
        io.setIntakeVoltage(voltage)
        recordOutput("Shooter/isOuttaking", true)
        recordOutput("Shooter/isIntaking", false)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}
