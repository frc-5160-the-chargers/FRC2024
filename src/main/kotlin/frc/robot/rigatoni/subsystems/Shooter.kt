package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import monologue.Annotations.Log
import monologue.Logged

private const val SHOOTER_MOTOR_ID = 7
private val CLOSED_LOOP_SPEAKER_SHOOT_SPEED = AngularVelocity(0.0) // tbd; should change soon depending on feedforward numbers

// standard: + = outtake, - = intake; regardless of voltage set
@Suppress("unused")
class Shooter(disable: Boolean = false): SubsystemBase(), Logged {
    private val motor: Motor = if (isSimulation() || disable) {
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

    @get:Log(key = "velocity(rad/s)")
    val angularVelocity get() = motor.encoder.angularVelocity

    fun setIdle(){ motor.stop() }

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
    }

    fun receiveFromSource(){
        setVoltage(-8.volts)
    }

    fun receiveFromGroundIntake(){
        setVoltage(5.volts)
    }

    fun setVoltage(voltage: Voltage){
        motor.voltageOut = voltage
        log("RequestedVoltage", voltage.inUnit(volts))
    }

    fun setSpeed(percentOut: Double){
        setVoltage(percentOut * 12.volts)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()) setIdle()
        log("MeasuredVoltage", motor.voltageOut.inUnit(volts))
        log("StatorCurrent", motor.statorCurrent.inUnit(amps))
    }
}