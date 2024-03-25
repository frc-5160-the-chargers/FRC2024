@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive.lowlevel

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.setVoltage

public class DiffDriveIOReal(
    logInputs: LoggableInputsProvider,
    private val topLeft: EncoderMotorController,
    private val topRight: EncoderMotorController,
    private val bottomLeft: EncoderMotorController,
    private val bottomRight: EncoderMotorController,
): DiffDriveIO {

    init {
        topLeft.inverted = false
        bottomLeft.inverted = false
        topRight.inverted = true
        bottomRight.inverted = true
    }

    private var leftAppliedVoltage = Voltage(0.0)
    private var rightAppliedVoltage = Voltage(0.0)

    override val leftWheelTravel: Angle by logInputs.quantity{
        (topLeft.encoder.angularPosition + bottomLeft.encoder.angularPosition) / 2.0
    }
    override val rightWheelTravel: Angle by logInputs.quantity{
        (topRight.encoder.angularPosition + bottomRight.encoder.angularPosition) / 2.0
    }

    override val leftVelocity: AngularVelocity by logInputs.quantity{
        (topLeft.encoder.angularVelocity + bottomLeft.encoder.angularVelocity) / 2.0
    }
    override val rightVelocity: AngularVelocity by logInputs.quantity{
        (topRight.encoder.angularVelocity + bottomRight.encoder.angularVelocity) / 2.0
    }

    override val leftVoltage: Voltage by logInputs.quantity{leftAppliedVoltage}
    override val rightVoltage: Voltage by logInputs.quantity {rightAppliedVoltage}

    override fun setVoltages(left: Voltage, right: Voltage) {
        leftAppliedVoltage = left
        rightAppliedVoltage = right
        // uses custom extension functions; see wpilibextensions
        topLeft.setVoltage(left)
        bottomLeft.setVoltage(left)
        topRight.setVoltage(right)
        bottomRight.setVoltage(right)
    }


    override var inverted: Boolean = false
        set(invertMotors){
            if (invertMotors) {
                topLeft.inverted = true
                bottomLeft.inverted = true
                topRight.inverted = false
                bottomRight.inverted = false
            }else{
                topLeft.inverted = false
                bottomLeft.inverted = false
                topRight.inverted = true
                bottomRight.inverted = true
            }
            field = invertMotors
        }
}