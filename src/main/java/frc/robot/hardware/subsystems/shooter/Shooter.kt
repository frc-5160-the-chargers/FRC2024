@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.feedforward.ArmFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.utils.within
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIO
import org.littletonrobotics.junction.Logger.recordOutput


enum class PivotAngle(val angle: Angle){
    AMP(0.degrees),
    SOURCE(0.degrees),
    SPEAKER(0.degrees),
    IDLE(0.degrees),
    GROUND_INTAKE_HANDOFF(0.degrees)
}


class Shooter(
    private val io: ShooterIO,
    private val pivotPID: PIDConstants,
    // SetpointSupplier.Default indicates no motion profile
    private val pivotProfile: AngularMotionProfile? = null,
    private val pivotFF: ArmFFEquation = ArmFFEquation(0.0,0.0,0.0),
    private val pivotPrecision: Precision.Within<AngleDimension> = Precision.Within(0.5.degrees)
): SubsystemBase() {

    private var targetPosition: Angle? = null
    private var motionProfileSetpoint = AngularMotionProfileState(io.pivotPosition)

    val hasHitPivotTarget: Boolean get() {
        val currentPosition = targetPosition ?: return true
        return currentPosition.within(pivotPrecision)
    }

    val hasGamepiece: Boolean get() = io.hasGamepiece

    val canDetectGamepieces: Boolean get() = io.hasBeamBreakSensor




    fun setIdle(){
        io.setPivotVoltage(0.volts)
        io.spin(0.volts, 0.volts)
    }
    
    fun setPivotPosition(target: PivotAngle){
        setPivotPosition(target.angle)
        recordOutput("Shooter/pivotAngleTarget", target)
    }

    fun setPivotPosition(position: Angle){
        val setpointPosition: Angle
        val feedforward: Voltage
        if (pivotProfile != null){
            motionProfileSetpoint = pivotProfile.calculate(
                0.02.seconds,
                motionProfileSetpoint,
                AngularMotionProfileState(position)
            )
            setpointPosition = motionProfileSetpoint.position
            feedforward = pivotFF(io.pivotPosition, motionProfileSetpoint.velocity)
        }else{
            setpointPosition = position
            feedforward = 0.volts
        }


        if (hasHitPivotTarget){
            io.setPivotVoltage(0.volts)
        }else{
            io.setPivotPosition(
                setpointPosition,
                pivotPID,
                feedforward
            )
        }
    }
    
    fun setPivotVoltage(voltage: Voltage) = io.setPivotVoltage(voltage)

    fun setPivotSpeed(percentOut: Double) = io.setPivotVoltage(percentOut * 11.volts)




    fun setSpeed(percentOut: Double){
        setVoltage(percentOut * 12.volts)
    }

    fun setVoltage(voltage: Voltage){
        if (io.hasGamepiece && RobotBase.isReal()){
            io.spin(0.volts, 0.volts)
        }else{
            io.spin(voltage, -voltage)
        }
    }




    fun setAngleCommand(target: PivotAngle): Command =
        buildCommand{
            runOnce(this@Shooter){ setPivotPosition(target) }

            loopUntil({ hasHitPivotTarget }, this@Shooter){
                setPivotPosition(target)
            }

            runOnce(this@Shooter){ setPivotVoltage(0.volts) }
        }
}