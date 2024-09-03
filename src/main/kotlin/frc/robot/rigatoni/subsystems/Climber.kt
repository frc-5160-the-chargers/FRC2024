package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax

private const val CLIMBER_ID_LEFT = 6
private const val CLIMBER_ID_RIGHT = 8

private const val GEAR_RATIO: Double = 10.0
private val HIGH_LIMIT: Angle? = -100.radians
private val LOW_LIMIT: Angle? = null
private val MAX_VOLTAGE: Voltage = 8.volts

class Climber: SuperSubsystem("Climber") {
    private val leftMotor: Motor = ChargerSparkMax(CLIMBER_ID_LEFT).configure(
        optimizeUpdateRate = true,
        velocityUpdateRate = 0.hertz,
        inverted = false,
        brakeWhenIdle = true,
        gearRatio = GEAR_RATIO
    )

    private val rightMotor: Motor = ChargerSparkMax(CLIMBER_ID_RIGHT).configure(
        optimizeUpdateRate = true,
        velocityUpdateRate = 0.hertz,
        inverted = true,
        brakeWhenIdle = true,
        gearRatio = GEAR_RATIO
    )

    private val leftPosition by logged{ leftMotor.encoder.angularPosition }
    private val rightPosition by logged{ rightMotor.encoder.angularPosition }

    private fun willSurpassLimit(hookSpeed: Double, position: Angle): Boolean{
        val surpassedUpperLimit: Boolean = HIGH_LIMIT != null && hookSpeed > 0.0 && position >= HIGH_LIMIT
        val surpassedLowerLimit: Boolean = LOW_LIMIT != null && hookSpeed < 0.0 && position <= LOW_LIMIT

        return surpassedUpperLimit || surpassedLowerLimit.also{ log("HasSurpassedLimit", it) }
    }

    fun moveLeftHook(speed: Double){
        if (willSurpassLimit(speed, leftPosition)){
            leftMotor.appliedVoltage = 0.volts
            log("LeftRequestedPercentOut", 0.0)
        }else{
            leftMotor.appliedVoltage = speed * MAX_VOLTAGE
            log("LeftRequestedPercentOut", speed)
        }
    }

    fun moveRightHook(speed: Double){
        if (willSurpassLimit(speed, rightPosition)){
            rightMotor.appliedVoltage = 0.volts
            log("RightRequestedPercentOut", 0.0)
        }else{
            rightMotor.appliedVoltage = speed * MAX_VOLTAGE
            log("RightRequestedPercentOut", speed)
        }
    }

    fun setIdle(){
        moveLeftHook(0.0)
        moveRightHook(0.0)
    }

    override fun periodic() {
        log("LeftVoltageReading", leftMotor.appliedVoltage)
        log("RightVoltageReading", rightMotor.appliedVoltage)
    }
}