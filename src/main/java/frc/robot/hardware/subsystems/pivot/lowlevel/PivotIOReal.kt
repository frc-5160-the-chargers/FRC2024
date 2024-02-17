package frc.robot.hardware.subsystems.pivot.lowlevel

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder

@Suppress("unused")
class PivotIOReal(
    private val motor: SmartEncoderMotorController,
    private val absoluteEncoder: PositionEncoder = object: PositionEncoder{ override val angularPosition = Angle(0.0) },
    useOnboardPID: Boolean = true,
    private val gearRatio: Double = 1.0,
    private val offset: Angle = Angle(0.0)
): PivotIO {

    private val rioController: SuperPIDController<AngleDimension, VoltageDimension>? =
        if (useOnboardPID){
            null
        }else{
            getRioPIDController()
        }

    override val appliedVoltage by PivotLog.quantity{
        motor.appliedVoltage
    }
    
    override val position by PivotLog.quantity{
        (motor.encoder.angularPosition / gearRatio) - offset
    }
    
    override val appliedCurrent by PivotLog.quantity{
        motor.appliedCurrent
    }

    override val absolutePosition by PivotLog.quantity{
        absoluteEncoder.angularPosition
    }
    
    override val tempCelsius by PivotLog.double{
        motor.tempCelsius
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setVoltage(voltage.siValue)
    }
    
    override fun setPositionSetpoint(position: Angle, pidConstants: PIDConstants, ffOutput: Voltage) {
        if (rioController != null){
            rioController.constants = pidConstants
            setVoltage(rioController.calculateOutput(position) + ffOutput)
        }else{
            motor.setAngularPosition(position, pidConstants, extraVoltage = ffOutput)
        }
    }
}