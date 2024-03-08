@file:Suppress("unused")
package frc.robot.hardware.subsystems.newclimber

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.newclimber.lowlevel.ClimberIO


private val CLIMB_UP_VOLTAGE = 8.volts
private val CLIMB_DOWN_VOLTAGE = (-8).volts


class Climber(
    private val io: ClimberIO,
    private val highLimit: Angle? = null,
    private val lowLimit: Angle? = null
): SubsystemBase() {

    fun moveLeftHookUp(voltage: Voltage = CLIMB_UP_VOLTAGE){
        if (highLimit != null && voltage > 0.volts && io.leftPosition >= highLimit){
            io.setLeftVoltage(0.volts)
        }else{
            io.setLeftVoltage(voltage)
        }
    }

    fun moveRightHookUp(voltage: Voltage = CLIMB_UP_VOLTAGE){
        if (highLimit != null && voltage > 0.volts && io.rightPosition >= highLimit){
            io.setRightVoltage(0.volts)
        }else{
            io.setRightVoltage(voltage)
        }
    }

    fun moveLeftHookDown(voltage: Voltage = CLIMB_DOWN_VOLTAGE){
        if (lowLimit != null && voltage < 0.volts && io.leftPosition <= lowLimit){
            io.setLeftVoltage(0.volts)
        }else{
            io.setLeftVoltage(voltage)
        }
    }

    fun moveRightHookDown(voltage: Voltage = CLIMB_DOWN_VOLTAGE){
        if (lowLimit != null && voltage < 0.volts && io.rightPosition <= lowLimit){
            io.setRightVoltage(0.volts)
        }else{
            io.setRightVoltage(voltage)
        }
    }

    fun setIdle(){
        io.setLeftVoltage(0.volts)
        io.setRightVoltage(0.volts)
    }
}