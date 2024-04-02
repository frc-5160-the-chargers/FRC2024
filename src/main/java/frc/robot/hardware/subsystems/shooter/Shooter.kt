@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIO
import org.littletonrobotics.junction.Logger.recordOutput

private const val WRONG_USAGE_OF_OUTTAKE_WARNING_MSG = "Applied voltage must be > 0.volts. To intake, call intake(voltage) or intake(speed) instead."

// standard: + = outtake, - = intake; regardless of voltage set
class Shooter(
    private val io: ShooterIO,
    private val shootingFFEquation: AngularMotorFFEquation,
    private val shootingPID: PIDConstants
): SubsystemBase() {
    private var wasShootingInSpeaker = false

    val hasNoteDetector: Boolean get() = io.hasNoteDetector

    val hasNote: Boolean get() = io.hasNote

    fun setIdle(){
        io.setIntakeVoltage(0.volts)
        wasShootingInSpeaker = false
    }

    fun outtakeAtAmpSpeed(){
        outtake(7.volts)
    }

    fun outtakeAtSpeakerSpeed() {
        val targetVelocity = AngularVelocity(0.0) // tbd; should change soon depending on feedforward numbers
        io.setVelocity(
            targetVelocity,
            shootingPID,
            shootingFFEquation(targetVelocity)
        )
        recordOutput("Shooter/isOuttaking", true)
        recordOutput("Shooter/isIntaking", false)
        if (!wasShootingInSpeaker && RobotBase.isSimulation()){
            wasShootingInSpeaker = true
            NoteVisualizer.shootInSpeakerCommand().schedule()
        }
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
        wasShootingInSpeaker = false
    }

    fun outtake(percentOut: Double){
        outtake(percentOut * 12.volts)
    }

    fun outtake(voltage: Voltage){
        io.setIntakeVoltage(voltage)
        if (voltage >= 0.volts){
            if (RobotBase.isSimulation()){
                error(WRONG_USAGE_OF_OUTTAKE_WARNING_MSG)
            }else{
                println(WRONG_USAGE_OF_OUTTAKE_WARNING_MSG)
            }
        }
        NoteVisualizer.setHasNote(false)
        recordOutput("Shooter/isOuttaking", true)
        recordOutput("Shooter/isIntaking", false)
        wasShootingInSpeaker = false
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}
