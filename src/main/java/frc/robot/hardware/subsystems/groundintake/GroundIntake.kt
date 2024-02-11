package frc.robot.hardware.subsystems.groundintake

import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIO

/**
 * Spins both the ground intake and the conveyor to pass to the shooter.
 */
class GroundIntake(io: GroundIntakeIO): SubsystemBase(), GroundIntakeIO by io { // implements GroundIntakeIO to inherit necessary functions from io layer
    fun intake(power: Double){
        intake(power * 12.volts)
    }

    fun setIdle(){
        intake(0.0)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}