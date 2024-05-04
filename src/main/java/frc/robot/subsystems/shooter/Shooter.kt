package frc.robot.subsystems.shooter

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.MotorizedComponent

private val CLOSED_LOOP_SPEAKER_SHOOT_SPEED = AngularVelocity(0.0) // tbd; should change soon depending on feedforward numbers

// standard: + = outtake, - = intake; regardless of voltage set
@Suppress("unused")
class Shooter(
    private val motor: MotorizedComponent,
    private val gearRatio: Double,
    private val shootingFFEquation: AngularMotorFFEquation = AngularMotorFFEquation(0, 0, 0),
    private val shootingPID: PIDConstants = PIDConstants(0,0,0),
    private val closedLoopSpeakerShooting: Boolean = true,
): SuperSubsystem("Shooter") {
    private var wasShootingInSpeaker = false

    val angularVelocity by logged{ motor.encoder.angularVelocity / gearRatio }

    fun setIdle(){
        motor.stop()
        wasShootingInSpeaker = false
    }

    fun outtakeAtAmpSpeed(){
        setVoltage(9.5.volts)
    }

    fun outtakeAtSpeakerSpeed() {
        if (closedLoopSpeakerShooting){
            motor.setVelocitySetpoint(
                CLOSED_LOOP_SPEAKER_SHOOT_SPEED * gearRatio,
                shootingPID,
                shootingFFEquation(CLOSED_LOOP_SPEAKER_SHOOT_SPEED)
            )
        }else{
            setVoltage(12.volts)
        }
        if (!wasShootingInSpeaker && RobotBase.isSimulation()){
            wasShootingInSpeaker = true
            NoteVisualizer.shootInSpeakerCommand().schedule()
        }
    }

    fun receiveFromSource(){
        setVoltage(-8.volts)
    }

    fun receiveFromGroundIntake(){
        setVoltage(5.volts)
    }

    fun setVoltage(voltage: Voltage){
        motor.appliedVoltage = voltage
        log("RequestedVoltage", voltage)
    }

    fun setSpeed(percentOut: Double){
        setVoltage(percentOut * 12.volts)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
        log("MeasuredVoltage", motor.appliedVoltage)
        log("StatorCurrent", motor.statorCurrent)
    }
}