@file:Suppress("unused")
package frc.robot.hardware.subsystems.climber

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIO






class Climber(
    private val io: ClimberIO,
    private val climbUpMaxVoltage: Voltage = 8.volts,
    private val climbDownMaxVoltage: Voltage = (-8).volts,
    private val highLimit: Angle? = null,
    private val lowLimit: Angle? = null
): SubsystemBase() {
    private fun surpassedLimit(hookSpeed: Double, position: Angle): Boolean{
        val surpassedUpperLimit: Boolean = highLimit != null && hookSpeed > 0.0 && position >= highLimit
        val surpassedLowerLimit: Boolean = highLimit != null && hookSpeed < 0.0 && position >= highLimit

        return surpassedUpperLimit || surpassedLowerLimit
    }

    fun moveLeftHook(speed: Double){
        if (speed == 0.0 || surpassedLimit(speed, io.leftPosition)){
            io.setLeftVoltage(0.volts)
        }else{
            io.setLeftVoltage(speed * climbUpMaxVoltage)
        }
    }

    fun moveRightHook(speed: Double){
        if (speed == 0.0 || surpassedLimit(speed, io.rightPosition)){
            io.setRightVoltage(0.volts)
        }else{
            io.setRightVoltage(speed * climbUpMaxVoltage)
        }
    }
}