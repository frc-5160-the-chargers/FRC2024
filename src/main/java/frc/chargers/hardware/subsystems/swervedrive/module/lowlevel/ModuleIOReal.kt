@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.subsystems.swervedrive.module.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.RobotController
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.DEFAULT_GEAR_RATIO
import frc.chargers.hardware.motorcontrol.*
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.Alert

/**
 * Represents the low level hardware of a SwerveModule on the real robot.
 */
public class ModuleIOReal(
    logInputs: LoggableInputsProvider,
    private val turnMotor: EncoderMotorController,
    private val turnEncoder: PositionEncoder,
    private val driveMotor: EncoderMotorController,
    private val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    private val turnGearRatio: Double = DEFAULT_GEAR_RATIO
): ModuleIO {
    private val startingWheelTravel = driveMotor.encoder.angularPosition

    private val batteryVoltageIssueAlert = Alert.warning(
        text = "It seems that the battery voltage from the Robot controller is being reported as extremely low(possibly 0)."
    )

    override var turnVoltage: Voltage by logInputs.quantity(
        getValue = {if (turnMotor is SmartEncoderMotorController) turnMotor.appliedVoltage else 0.volts},
        setValue = {
            turnMotor.setVoltage(it.coerceIn(getVoltageRange()))
        }
    )

    override var driveVoltage: Voltage by logInputs.quantity(
        getValue = {if (driveMotor is SmartEncoderMotorController) driveMotor.appliedVoltage else 0.volts},
        setValue = {
            driveMotor.setVoltage(it.coerceIn(getVoltageRange()))
        }
    )

    override val logTab: String = logInputs.namespace

    override val direction: Angle by logInputs.quantity{
        turnEncoder.angularPosition.inputModulus(0.degrees..360.degrees)
    }

    override val turnSpeed: AngularVelocity by logInputs.quantity{
        turnMotor.encoder.angularVelocity / turnGearRatio
    }

    override val speed: AngularVelocity by logInputs.quantity{
        driveMotor.encoder.angularVelocity / driveGearRatio
    }

    override val wheelTravel: Angle by logInputs.quantity{
        (driveMotor.encoder.angularPosition - startingWheelTravel) / driveGearRatio
    }

    override val driveCurrent: Current by logInputs.quantity{
        if (driveMotor is SmartEncoderMotorController){
            driveMotor.appliedCurrent
        }else{
            Current(0.0)
        }
    }

    override val turnCurrent: Current by logInputs.quantity{
        if (turnMotor is SmartEncoderMotorController){
            turnMotor.appliedCurrent
        }else{
            Current(0.0)
        }
    }

    override val driveTempCelsius: Double by logInputs.double{
        if (driveMotor is SmartEncoderMotorController){
            driveMotor.tempCelsius
        }else{
            0.0
        }
    }

    override val turnTempCelsius: Double by logInputs.double{
        if (turnMotor is SmartEncoderMotorController){
            turnMotor.tempCelsius
        }else{
            0.0
        }
    }

    private fun getVoltageRange(): ClosedRange<Voltage>{
        val upperLimit = RobotController.getBatteryVoltage().ofUnit(volts)
        return if (upperLimit < 1.volts){
            batteryVoltageIssueAlert.active = true
            (-12).volts..12.volts
        }else{
            -upperLimit..upperLimit
        }
    }

}