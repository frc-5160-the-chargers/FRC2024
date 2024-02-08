package frc.robot.hardware.subsystems.groundintake

import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GroundIntake(io: GroundIntakeIO): SubsystemBase(), GroundIntakeIO by io { // implements GroundIntakeIO to inherit necessary functions from io layer
    fun setSpeed(power: Double){
        setVoltage(power * 12.volts)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setVoltage(0.volts)
        }
    }
}