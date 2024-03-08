package frc.robot.hardware.subsystems.groundintake

import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIO

/**
 * Spins both the ground intake and the conveyor to pass to the shooter.
 */
// standard: + = outtake, - = intake for both conveyor and ground intake components
class GroundIntakeSerializer(io: GroundIntakeIO): SubsystemBase(), GroundIntakeIO by io{ // implements GroundIntakeIO to inherit necessary functions from io layer

    fun setIdle(){
        setIntakeVoltage(0.volts)
        setConveyorVoltage(0.volts)
    }

    fun intake(){
        setIntakeVoltage(-8.volts)
        setConveyorVoltage(-8.volts)
    }

    fun outtake(){
        setIntakeVoltage(8.volts)
        setConveyorVoltage(8.volts)
    }

    fun passToShooter(){
        setConveyorVoltage(-8.volts)
        setIntakeVoltage(-6.volts) // intakes a little just in case the note is still in the ground intake portion
    }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}