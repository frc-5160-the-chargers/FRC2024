package frc.robot.subsystems

import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.MotorizedComponent


/**
 * Spins both the ground intake and the conveyor to pass to the shooter.
 */
// standard: + = outtake, - = intake for both conveyor and ground intake components
// first list value is ground intake motor; second is serialier motor.
@Suppress("unused")
class GroundIntakeSerializer(
    private val groundIntakeMotor: MotorizedComponent,
    private val serializerMotor: MotorizedComponent? = null,
    private val groundIntakeGearRatio: Double,
    private val serializerGearRatio: Double,
): SuperSubsystem("GroundIntakeSerializer"){ // implements GroundIntakeIO to inherit necessary functions from io layer
    val statorCurrentReadings by logged{
        listOfNotNull(groundIntakeMotor.statorCurrent, serializerMotor?.statorCurrent)
    }

    val voltageReadings by logged{
        listOfNotNull(groundIntakeMotor.appliedVoltage, serializerMotor?.appliedVoltage)
    }

    var requestedVoltages by logged(mutableListOf(0.volts, 0.volts))

    val angularVelocityReadings by logged{
        val readings = mutableListOf(groundIntakeMotor.encoder.angularVelocity / groundIntakeGearRatio)
        if (serializerMotor != null){
            readings.add(serializerMotor.encoder.angularVelocity / serializerGearRatio)
        }
        readings
    }

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
    }
}
