package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.HorseLog.log
import frc.chargers.framework.logged
import frc.chargers.framework.tunable
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ChargerSparkMax

private const val CLIMBER_ID_LEFT = 6
private const val CLIMBER_ID_RIGHT = 8

private const val GEAR_RATIO = 10.0
private val MAX_VOLTAGE = 8.volts

class Climber: SubsystemBase() {
    private val highLimit by tunable(Angle(Double.POSITIVE_INFINITY))
    private val lowLimit by tunable(Angle(Double.NEGATIVE_INFINITY))

    private val leftMotor: Motor = ChargerSparkMax(CLIMBER_ID_LEFT, faultLogName = "LeftClimberMotor")
        .configure(
            optimizeUpdateRate = true,
            velocityUpdateRate = 0.hertz,
            inverted = false,
            brakeWhenIdle = true,
            gearRatio = GEAR_RATIO
        )

    private val rightMotor: Motor = ChargerSparkMax(CLIMBER_ID_RIGHT, faultLogName = "RightClimberMotor")
        .configure(
            optimizeUpdateRate = true,
            velocityUpdateRate = 0.hertz,
            inverted = true,
            brakeWhenIdle = true,
            gearRatio = GEAR_RATIO
        )

    private val leftPosition by logged{ leftMotor.encoder.angularPosition }
    private val rightPosition by logged{ rightMotor.encoder.angularPosition }

    private fun willSurpassLimit(hookSpeed: Double, position: Angle): Boolean{
        val surpassedUpperLimit = hookSpeed > 0.0 && position >= highLimit
        val surpassedLowerLimit = hookSpeed < 0.0 && position <= lowLimit
        return (surpassedUpperLimit || surpassedLowerLimit).also { log("Climber/HasSurpassedLimit", it) }
    }

    fun moveLeftHook(speed: Double){
        if (willSurpassLimit(speed, leftPosition)){
            leftMotor.voltageOut = 0.volts
            log("Climber/LeftRequest", 0.0)
        }else{
            leftMotor.voltageOut = speed * MAX_VOLTAGE
            log("Climber/LeftRequest", speed)
        }
    }

    fun moveRightHook(speed: Double){
        if (willSurpassLimit(speed, rightPosition)){
            rightMotor.voltageOut = 0.volts
            log("Climber/RightRequest", 0.0)
        }else{
            rightMotor.voltageOut = speed * MAX_VOLTAGE
            log("Climber/RightRequest", speed)
        }
    }

    fun setIdle(){
        moveLeftHook(0.0)
        moveRightHook(0.0)
    }

    override fun periodic() {
        log("Climber/LeftV", leftMotor.voltageOut)
        log("Climber/RightV", rightMotor.voltageOut)
    }
}