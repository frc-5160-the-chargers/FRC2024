package frc.robot.rigatoni.subsystems.shooter

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.simulation.MotorSim

private const val SHOOTER_MOTOR_ID = 7
private val CLOSED_LOOP_SPEAKER_SHOOT_SPEED = AngularVelocity(0.0) // tbd; should change soon depending on feedforward numbers

// standard: + = outtake, - = intake; regardless of voltage set
@Suppress("unused")
class Shooter: SuperSubsystem("Shooter") {
    private val motor: Motor = if (isSimulation()) {
        MotorSim(DCMotor.getNeoVortex(1))
    } else {
        ChargerSparkFlex(SHOOTER_MOTOR_ID).configure(inverted = true)
    }

    init {
        motor.configure(
            optimizeUpdateRate = true,
            statorCurrentLimit = 60.amps,
            gearRatio = 1.698,
            velocityPID = PIDConstants(0.2, 0.0, 0.0)
        )
    }

    private val shootingFFEquation = AngularMotorFFEquation(0.0, 0.0, 0.0)
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
                shootingFFEquation.calculate(CLOSED_LOOP_SPEAKER_SHOOT_SPEED)
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