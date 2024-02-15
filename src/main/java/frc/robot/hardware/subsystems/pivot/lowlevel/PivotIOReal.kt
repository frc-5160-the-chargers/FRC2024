package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import frc.chargers.constants.DEFAULT_GEAR_RATIO
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController

@Suppress("unused")
class PivotIOReal(
    private val motor: SmartEncoderMotorController,
    private val gearRatio: Double = DEFAULT_GEAR_RATIO,
    private val offset: Angle = Angle(0.0)
): PivotIO {
    override val appliedVoltage by PivotLog.quantity{
        motor.appliedVoltage
    }
    
    override val position by PivotLog.quantity{
        (motor.encoder.angularPosition / gearRatio) - offset
    }
    
    override val appliedCurrent by PivotLog.quantity{
        motor.appliedCurrent
    }
    
    override val tempCelsius by PivotLog.double{
        motor.tempCelsius
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.siValue)
    }
    
    override fun setPosition(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        motor.setAngularPosition(position, pidConstants, extraVoltage = ffOutput)
    }
}