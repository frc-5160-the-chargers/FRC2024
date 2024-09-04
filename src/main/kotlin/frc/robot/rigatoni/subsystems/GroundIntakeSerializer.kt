package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot.isSimulation
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.simulation.MotorSim

private const val SERIALIZER_ID = 29
private const val GROUND_INTAKE_ID = 7

// standard: + = outtake, - = intake for both conveyor and ground intake components
// first list value is ground intake motor; second is serializer motor.
class GroundIntakeSerializer: SuperSubsystem("GroundIntakeSerializer"){
    private val groundIntakeMotor: Motor
    private val serializerMotor: Motor?

    init {
        if (isSimulation()){
            groundIntakeMotor = MotorSim(DCMotor.getFalcon500(1))
            serializerMotor = MotorSim(DCMotor.getNEO(1))
        } else {
            groundIntakeMotor = ChargerTalonFX(GROUND_INTAKE_ID)
            serializerMotor = ChargerSparkMax(SERIALIZER_ID).configure(inverted = true)
        }

        groundIntakeMotor.configure(
            optimizeUpdateRate = true,
            gearRatio = 15.0 / 12.0
        )
        serializerMotor?.configure(
            optimizeUpdateRate = true,
            statorCurrentLimit = 45.amps,
            gearRatio = 7.5 / 1.0
        )
    }

    private var requestedVoltages by logged(mutableListOf(0.volts, 0.volts))

    fun setIntakeVoltage(voltage: Voltage){
        groundIntakeMotor.appliedVoltage = voltage
        requestedVoltages[0] = voltage
    }

    fun setConveyorVoltage(voltage: Voltage){
        serializerMotor?.appliedVoltage = voltage
        requestedVoltages[1] = voltage
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
        if (DriverStation.isDisabled()){
            setIdle()
        }
        log("StatorCurrentReadings", listOfNotNull(groundIntakeMotor.statorCurrent, serializerMotor?.statorCurrent))
        log("VoltageReadings", listOfNotNull(groundIntakeMotor.appliedVoltage, serializerMotor?.appliedVoltage))
        log("AngularVelocityReadings", listOfNotNull(groundIntakeMotor.encoder.angularVelocity, serializerMotor?.encoder?.angularVelocity))
    }
}
