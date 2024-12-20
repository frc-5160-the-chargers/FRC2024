package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot.isSimulation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.HorseLog.log
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.simulation.MotorSim

private const val SERIALIZER_ID = 29
private const val GROUND_INTAKE_ID = 7

// standard: + = outtake, - = intake for both conveyor and ground intake components
// first list value is ground intake motor; second is serializer motor.
class GroundIntakeSerializer(disable: Boolean = false): SubsystemBase() {
    private val groundIntakeMotor: Motor
    private val serializerMotor: Motor

    init {
        if (isSimulation() || disable){
            groundIntakeMotor = MotorSim(DCMotor.getFalcon500(1))
            serializerMotor = MotorSim(DCMotor.getNEO(1))
        } else {
            groundIntakeMotor = ChargerTalonFX(GROUND_INTAKE_ID, faultLogName = "GroundIntakeMotor")
                .limitSupplyCurrent(30.amps, highLimit = 60.amps, highLimitAllowedFor = 1.seconds)
            serializerMotor = ChargerSparkMax(SERIALIZER_ID, faultLogName = "SerializerMotor")
                .configure(inverted = true)
        }
        groundIntakeMotor.configure(
            //optimizeUpdateRate = true,
            gearRatio = 15.0 / 12.0,
            statorCurrentLimit = 60.amps
        )
        serializerMotor.configure(
            //optimizeUpdateRate = true,
            statorCurrentLimit = 45.amps,
            gearRatio = 7.5 / 1.0
        )
    }

    fun setIntakeVoltage(voltage: Voltage){
        groundIntakeMotor.voltageOut = voltage
        log("GroundIntakeSerializer/Voltage/Intake", voltage)
    }

    fun setConveyorVoltage(voltage: Voltage){
        serializerMotor.voltageOut = voltage
        log("GroundIntakeSerializer/Voltage/Serializer", voltage)
    }

    fun setIdle(){
        setIntakeVoltage(0.volts)
        setConveyorVoltage(0.volts)
    }

    fun intake(){
        setIntakeVoltage(-11.volts)
        setConveyorVoltage(-12.volts)
    }

    fun outtake(){
        setIntakeVoltage(9.volts)
        setConveyorVoltage(11.volts)
    }

    fun passToShooterSlow(){
        setConveyorVoltage(-10.volts)
        setIntakeVoltage(-6.volts) // intakes a little just in case the note is still in the ground intake portion
    }

    fun passToShooterFast(){
        setConveyorVoltage(-12.volts)
        setIntakeVoltage(-6.volts)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()) setIdle()
        log("GroundIntakeSerializer/StatorCurrent/Intake", groundIntakeMotor.statorCurrent)
        log("GroundIntakeSerializer/StatorCurrent/Serializer", serializerMotor.statorCurrent)
        log("GroundIntakeSerializer/AngularVelocity/Intake", groundIntakeMotor.encoder.angularVelocity)
        log("GroundIntakeSerializer/AngularVelocity/Serializer", serializerMotor.encoder.angularVelocity)
    }
}
