@file:Suppress("unused")
package frc.robot.hardware.subsystems.climber

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.utils.Precision
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIO


// change this to voltage requests / positions requests
// and/or make it stall in the same position when no power is put
class Climber(
    private val io: ClimberIO,
    private val climbMaxVoltage: Voltage = 8.volts,
    val highLimit: Angle? = null,
    val lowLimit: Angle? = null,
    private val climbSetpointPrecision: Precision<AngleDimension> = Precision.Within(2.degrees)
): SubsystemBase() {
    /*
    private val leftController = SuperPIDController(
        climbPIDConstants,
        getInput = { io.leftPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts,
        selfSustain = true
    )

    private val rightController = SuperPIDController(
        climbPIDConstants,
        getInput = { io.rightPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts,
        selfSustain = true
    )

     */

    private fun willSurpassLimit(hookSpeed: Double, position: Angle): Boolean{
        val surpassedUpperLimit: Boolean = highLimit != null && hookSpeed > 0.0 && position >= highLimit
        val surpassedLowerLimit: Boolean = lowLimit != null && hookSpeed < 0.0 && position <= lowLimit

        return surpassedUpperLimit || surpassedLowerLimit
    }

    fun moveLeftHook(speed: Double){
        if (willSurpassLimit(speed, io.leftPosition)){
            io.setLeftVoltage(0.volts)
        }else{
            io.setLeftVoltage(speed * climbMaxVoltage)
        }
    }

    fun moveRightHook(speed: Double){
        if (willSurpassLimit(speed, io.rightPosition)){
            io.setRightVoltage(0.volts)
        }else{
            io.setRightVoltage(speed * climbMaxVoltage)
        }
    }

    fun setIdle(){
        moveLeftHook(0.0)
        moveRightHook(0.0)
    }
}