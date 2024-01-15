@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.utils.within


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
    private val pivotProfile: SetpointSupplier<AngleDimension, VoltageDimension> = SetpointSupplier.Default(),
    private val pivotPrecision: Precision.Within<AngleDimension> = Precision.Within(0.5.degrees)
): SubsystemBase() {

    private var targetPosition: Angle? = null

    fun hasHitPivotTarget(): Boolean{
        val currentPosition = targetPosition ?: return true
        return currentPosition.within(pivotPrecision)
    }

    fun setPivotPosition(target: PivotAngle){
        setPivotPosition(target.angle)
    }

    fun setPivotPosition(position: Angle){
        targetPosition = position
        val setpoint = pivotProfile.calculateSetpoint(position)
        if (hasHitPivotTarget()){
            io.setPivotVoltage(0.volts)
        }else{
            io.setPivotPosition(
                setpoint.value,
                pivotPID,
                setpoint.feedforwardOutput
            )
        }
    }

    fun spin(power: Double){
        spin(power * 12.volts)
    }

    fun spin(voltage: Voltage){
        if (io.hasGamepiece && RobotBase.isReal()){
            io.spin(0.volts, 0.volts)
        }else{
            io.spin(voltage, -voltage)
        }
    }
}