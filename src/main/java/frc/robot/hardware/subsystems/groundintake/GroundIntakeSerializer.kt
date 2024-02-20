package frc.robot.hardware.subsystems.groundintake

import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.wpilibextensions.Alert
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIO
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

/**
 * Spins both the ground intake and the conveyor to pass to the shooter.
 */
// standard: + = outtake, - = intake
class GroundIntakeSerializer(io: GroundIntakeIO): SubsystemBase(), GroundIntakeIO by io{ // implements GroundIntakeIO to inherit necessary functions from io layer
    private val passToShooterAlert =
        Alert.warning(text = "You are attempting to pass a ground intaked piece to the shooter; however, the shooter already has a gamepiece.")

    fun setIdle(){
        setIntakeVoltage(0.volts)
        setConveyorVoltage(0.volts)
    }

    /**
     * Intakes a note with the help of a pivot;
     * stowing it within the serializer.
     *
     * To pass the note to the shooter, you should use intakeToShooter() instead,
     * or
     */
    fun intake(pivot: Pivot){
        pivot.setAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
        if (abs(pivot.angle - PivotAngle.GROUND_INTAKE_HANDOFF) < 10.degrees) {
            setIntakeVoltage((-10).volts)
            setConveyorVoltage(6.volts)
        }
    }


    /**
     * Passes a stowed gamepiece within the conveyor/serializer
     * to the shooter.
     *
     * You should only call this method if a note is stowed within the serializer/conveyor;
     * if not, do not call this.
     */
    fun passToShooter(shooter: Shooter){
        if (shooter.hasBeamBreakSensor){
            if (shooter.hasNote){
                passToShooterAlert.active = true
                println(passToShooterAlert.text)
                return
            }
            shooter.intake(6.volts)
        }else{
            shooter.intake(2.volts)
        }
        setConveyorVoltage(6.volts)
    }

    /**
     * Outtakes; useful for ferrying notes(niche usecase).
     */
    fun outtake(){
        setIntakeVoltage(8.volts)
        setConveyorVoltage((-6).volts)
    }


    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}