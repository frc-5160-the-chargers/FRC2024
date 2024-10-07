package frc.robot.rigatoni.subsystems.shooter

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.HorseLog.log
import frc.chargers.framework.logged
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.simulation.MotorSim

private const val SHOOTER_MOTOR_ID = 7
private val CLOSED_LOOP_SPEAKER_SHOOT_SPEED = AngularVelocity(0.0) // tbd; should change soon depending on feedforward numbers

// standard: + = outtake, - = intake; regardless of voltage set
@Suppress("unused")
class Shooter: SubsystemBase() {
    private val motor: Motor = if (isSimulation()) {
        MotorSim(DCMotor.getNeoVortex(1))
    } else {
        ChargerSparkFlex(SHOOTER_MOTOR_ID, faultLogName = "ShooterMotor")
            .configure(inverted = true)
    }

    init {
        motor.configure(
            optimizeUpdateRate = true,
            statorCurrentLimit = 60.amps,
            gearRatio = 1.698,
            velocityPID = PIDConstants(0.2, 0.0, 0.0)
        )
    }

    private val shootingFFEquation = AngularMotorFeedforward(0.0, 0.0, 0.0)
    private val closedLoopSpeakerShooting = false
    private var wasShootingInSpeaker = false

    val angularVelocity by logged{ motor.encoder.angularVelocity }

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
                CLOSED_LOOP_SPEAKER_SHOOT_SPEED,
                shootingFFEquation(CLOSED_LOOP_SPEAKER_SHOOT_SPEED)
            )
        }else{
            setVoltage(12.volts)
        }
        if (!wasShootingInSpeaker && isSimulation()){
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
        motor.voltageOut = voltage
        log("Shooter/RequestedVoltage", voltage)
    }

    fun setSpeed(percentOut: Double){
        setVoltage(percentOut * 12.volts)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()) setIdle()
        log("Shooter/MeasuredVoltage", motor.voltageOut)
        log("Shooter/StatorCurrent", motor.statorCurrent)
    }
}