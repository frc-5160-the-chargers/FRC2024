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
    fun setIdle(){
        intake(0.volts)
    }

    fun intake(){
        intake(0.5)
    }

    fun outtake(){
        intake(-0.3)
    }

    fun intake(percentOut: Double){
        intake(percentOut * 12.volts)
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}