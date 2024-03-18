package frc.chargers.hardware.subsystems.swervedrive.modulelowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.RobotController
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.advantagekitextensions.recordOutput
import frc.chargers.constants.DEFAULT_GEAR_RATIO
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.Alert

/**
 * Represents a swerve module on a real robot,
 * encapsulating its low-level functionality
 */
class ModuleIOReal(
    logInputs: LoggableInputsProvider,
    private val useOnboardPID: Boolean = false,
    private val turnMotor: EncoderMotorController,
    private val turnEncoder: PositionEncoder,
    private val driveMotor: EncoderMotorController,
    private val turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    private val driveGearRatio: Double = DEFAULT_GEAR_RATIO,
    couplingRatio: Double? = null
): ModuleIO() {
    private val startingWheelTravel = driveMotor.encoder.angularPosition

    private val batteryVoltageIssueAlert = Alert.warning(
        text = "It seems that the battery voltage from the Robot controller is being reported as extremely low(possibly 0)."
    )

    override val turnVoltage by logInputs.quantity{
        if (turnMotor is SmartEncoderMotorController){
            turnMotor.appliedVoltage
        } else{
            0.volts
        }
    }

    override val driveVoltage by logInputs.quantity{
        if (driveMotor is SmartEncoderMotorController){
            driveMotor.appliedVoltage
        } else{
            0.volts
        }
    }

    override val direction by logInputs.quantity{
        turnEncoder.angularPosition.inputModulus(0.degrees..360.degrees)
    }

    override val turnSpeed by logInputs.quantity{
        turnMotor.encoder.angularVelocity / turnGearRatio
    }

    override val speed by logInputs.quantity{
        driveMotor.encoder.angularVelocity / driveGearRatio
    }

    override val wheelTravel by logInputs.quantity{
        (driveMotor.encoder.angularPosition + couplingOffset - startingWheelTravel) / driveGearRatio
    }

    override val driveCurrent by logInputs.quantity{
        if (driveMotor is SmartEncoderMotorController){
            driveMotor.appliedCurrent
        }else{
            Current(0.0)
        }
    }

    override val turnCurrent by logInputs.quantity{
        if (turnMotor is SmartEncoderMotorController){
            turnMotor.appliedCurrent
        }else{
            Current(0.0)
        }
    }

    private val startingTurnEncoderPosition = direction
    private var couplingOffset = 0.degrees

    init{
        if (useOnboardPID){
            require(driveMotor is SmartEncoderMotorController && turnMotor is SmartEncoderMotorController){
                "Your drive and turn motors must have onboard pid control available."
            }
            disableDefaultControllers()
        }

        if (couplingRatio != null){
            ChargerRobot.runPeriodically {
                couplingOffset -= couplingRatio * (direction - startingTurnEncoderPosition).inputModulus(-180.degrees..180.degrees)
                recordOutput(logInputs.namespace + "/couplingOffset", couplingOffset)
            }
        }
    }

    override fun setTurnVoltage(voltage: Voltage) {
        turnMotor.setVoltage(voltage.coerceIn(getVoltageRange()).siValue)
    }

    override fun setDriveVoltage(voltage: Voltage) {
        driveMotor.setVoltage(voltage.coerceIn(getVoltageRange()).siValue)
    }

    override fun setSpeedSetpoint(velocity: AngularVelocity, pidConstants: PIDConstants, feedforward: Voltage) {
        if (useOnboardPID && driveMotor is SmartEncoderMotorController){
            driveMotor.setAngularVelocity(
                velocity,
                pidConstants,
                feedforward
            )
        }else{
            super.setSpeedSetpoint(velocity, pidConstants, feedforward)
        }
    }

    override fun setDirectionSetpoint(direction: Angle, pidConstants: PIDConstants, feedforward: Voltage) {
        if (useOnboardPID && turnMotor is SmartEncoderMotorController){
            turnMotor.setAngularPosition(
                direction,
                pidConstants,
                continuousWrap = true,
                feedforward,
                turnEncoder,
                turnGearRatio,
            )
        }else{
            super.setDirectionSetpoint(direction, pidConstants, feedforward)
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