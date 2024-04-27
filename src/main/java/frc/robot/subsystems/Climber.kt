package frc.robot.subsystems


import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.MotorizedComponent


// change this to voltage requests / positions requests
// and/or make it stall in the same position when no power is put
@Suppress("unused")
class Climber(
    private val leftMotor: MotorizedComponent,
    private val rightMotor: MotorizedComponent,
    private val gearRatio: Double,
    private val climbMaxVoltage: Voltage = 8.volts,
    private val highLimit: Angle? = null,
    private val lowLimit: Angle? = null
): SuperSubsystem("Climber") {
    private val leftPosition by logged{ leftMotor.encoder.angularPosition / gearRatio }
    private val rightPosition by logged{ rightMotor.encoder.angularPosition / gearRatio }

    val leftVoltageReading by logged{ leftMotor.appliedVoltage }
    val rightVoltageReading by logged{ rightMotor.appliedVoltage }

    private fun willSurpassLimit(hookSpeed: Double, position: Angle): Boolean{
        val surpassedUpperLimit: Boolean = highLimit != null && hookSpeed > 0.0 && position >= highLimit
        val surpassedLowerLimit: Boolean = lowLimit != null && hookSpeed < 0.0 && position <= lowLimit

        return surpassedUpperLimit || surpassedLowerLimit.also{ log("HasSurpassedLimit", it) }
    }

    fun moveLeftHook(speed: Double){
        if (willSurpassLimit(speed, leftPosition)){
            leftMotor.appliedVoltage = 0.volts
            log("LeftRequestedPercentOut", 0.0)
        }else{
            leftMotor.appliedVoltage = speed * climbMaxVoltage
            log("LeftRequestedPercentOut", speed)
        }
    }

    fun moveRightHook(speed: Double){
        if (willSurpassLimit(speed, rightPosition)){
            rightMotor.appliedVoltage = 0.volts
            log("RightRequestedPercentOut", 0.0)
        }else{
            rightMotor.appliedVoltage = speed * climbMaxVoltage
            log("RightRequestedPercentOut", speed)
        }
    }

    fun setIdle(){
        moveLeftHook(0.0)
        moveRightHook(0.0)
    }

    override fun periodic(){

    }
}
