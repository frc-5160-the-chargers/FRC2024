package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import frc.chargers.controls.UnitPIDController
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.Loggable
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.angle
import kotlin.math.abs

class SwerveModule(
    override val namespace: String,
    private val turnMotor: Motor,
    // turn encoders are optional in sim
    private val turnEncoder: PositionEncoder? = null,
    private val driveMotor: Motor,
    private val moduleConstants: SwerveModuleConstants
): Loggable {
    private fun turnEncoderReading(): Angle =
        turnEncoder?.angularPosition ?: (turnMotor.encoder.angularPosition / moduleConstants.turnGearRatio)

    private val startingDriveEncoderPosition: Angle = driveMotor.encoder.angularPosition
    private val startingTurnRelativeEncoderPosition: Angle = turnMotor.encoder.angularPosition
    private val startingTurnAbsoluteEncoderPosition: Angle = turnEncoderReading().inputModulus(0.degrees..360.degrees)

    private val wheelRadius = moduleConstants.wheelDiameter / 2.0
    private val batteryVoltageIssueAlert = Alert.warning(
        text = "It seems that the battery voltage from the Robot controller is being reported as extremely low(possibly 0)."
    )

    private val rioAzimuthController = UnitPIDController<AngleDimension, VoltageDimension>(
        moduleConstants.turnMotorControlScheme.pidConstants,
        continuousInputRange = 0.degrees..360.degrees
    )
    private val rioVelocityController = UnitPIDController<AngularVelocityDimension, VoltageDimension>(moduleConstants.velocityPID)

    private var couplingOffset: Angle = 0.degrees
    private var azimuthProfileState = AngularMotionProfileState(startingTurnAbsoluteEncoderPosition)

    val direction: Angle by logged{
        turnEncoderReading().inputModulus(0.degrees..360.degrees)
    }
    @Suppress("unused")
    val turnAngularVelocity: AngularVelocity by logged{
        turnMotor.encoder.angularVelocity / moduleConstants.turnGearRatio
    }

    val driveAngularVelocity: AngularVelocity by logged{
        driveMotor.encoder.angularVelocity / moduleConstants.driveGearRatio
    }
    val driveLinearVelocity: Velocity by logged{
        driveAngularVelocity * moduleConstants.wheelDiameter / 2.0
    }
    val wheelTravel: Distance by logged{
        (driveMotor.encoder.angularPosition + couplingOffset - startingDriveEncoderPosition) /
                moduleConstants.driveGearRatio *
                wheelRadius
    }


    init{
        if (moduleConstants.turnMotorInverted && RobotBase.isReal()){
            turnMotor.hasInvert = !turnMotor.hasInvert
            log("Data/TurnMotorInverted", true)
        }else{
            log("Data/TurnMotorInverted", false)
        }

        log("Data/UsingCouplingRatio", moduleConstants.couplingRatio != null)

        ChargerRobot.runPeriodic {
            if (moduleConstants.couplingRatio != null){
                couplingOffset -= moduleConstants.couplingRatio * direction.inputModulus(-180.degrees..180.degrees)
                log("CouplingOffset", couplingOffset)
            }

            log("TurnMotorCurrent", turnMotor.statorCurrent)
            log("DriveMotorCurrent", driveMotor.statorCurrent)
        }
    }

    fun getModuleState(): SwerveModuleState =
        SwerveModuleState(
            driveLinearVelocity.inUnit(meters / seconds),
            Rotation2d(direction)
        )

    fun getModulePosition(): SwerveModulePosition =
        SwerveModulePosition(
            wheelTravel.inUnit(meters),
            Rotation2d(direction)
        )

    fun setDriveVoltage(target: Voltage){
        driveMotor.appliedVoltage = target
        log("DriveVoltage", target)
    }

    fun setTurnVoltage(target: Voltage){
        val trueVoltage = if (abs(target) < 0.01.volts){
            0.volts
        }else{
            target
        }
        turnMotor.appliedVoltage = trueVoltage
        log("TurnVoltage", trueVoltage)
    }

    fun setDirection(target: Angle){
        // utilizes custom absolute value overload for kmeasure quantities
        if (abs(direction - target).within(moduleConstants.turnMotorControlScheme.precision)){
            setTurnVoltage(0.volts)
            azimuthProfileState = AngularMotionProfileState(direction)
            return
        }

        val pidTarget: Angle
        val feedforwardV: Voltage

        when (moduleConstants.turnMotorControlScheme){
            is SwerveAzimuthControl.ProfiledPID -> {
                val goalState = AngularMotionProfileState(target)

                fun calculateSetpoint(): AngularMotionProfileState =
                    moduleConstants.turnMotorControlScheme.motionProfile.calculate(
                        setpoint = azimuthProfileState,
                        goal = goalState,
                        measurement = direction,
                        continuousInputRange = 0.degrees..360.degrees
                    )

                // increments the setpoint by calculating a new one
                azimuthProfileState = calculateSetpoint()

                // Calculates the setpoint 1 loop period in the future,
                // in order to do plant inversion feedforward.
                val futureSetpoint = calculateSetpoint()

                feedforwardV = moduleConstants.turnMotorControlScheme.ffEquation.calculate(
                    azimuthProfileState.velocity,
                    futureSetpoint.velocity
                )
                pidTarget = azimuthProfileState.position
            }

            is SwerveAzimuthControl.PID -> {
                pidTarget = target
                feedforwardV = 0.volts
            }
        }

        // uses pid control to move to the calculated setpoint, to achieve the target goal.
        if (moduleConstants.useOnboardPID){
            turnMotor.setPositionSetpoint(
                rawPosition = startingTurnRelativeEncoderPosition + (pidTarget - startingTurnAbsoluteEncoderPosition) * moduleConstants.turnGearRatio,
                moduleConstants.turnMotorControlScheme.pidConstants,
                feedforward = feedforwardV
            )
        }else{
            setTurnVoltage(
                (rioAzimuthController.calculate(direction, pidTarget) + feedforwardV).coerceIn(getVoltageRange())
            )
        }
    }

    fun setDesiredStateOpenLoop(state: SwerveModuleState){
        val directionAsRotation2d = Rotation2d(direction)
        val optimizedState = SwerveModuleState.optimize(state, directionAsRotation2d)

        optimizedState.speedMetersPerSecond *= abs(
            (optimizedState.angle - directionAsRotation2d).cos
        )

        setDirection(optimizedState.angle.angle)
        setDriveVoltage(
            (optimizedState.speedMetersPerSecond /
                    moduleConstants.driveMotorMaxSpeed.inUnit(meters / seconds) *
                    12.volts).coerceIn(getVoltageRange())
        )
    }

    fun setDesiredStateClosedLoop(state: SwerveModuleState){
        val directionAsRotation2d = Rotation2d(direction)
        val optimizedState = SwerveModuleState.optimize(state, directionAsRotation2d)

        setDirection(optimizedState.angle.angle)

        val velocitySetpoint = optimizedState.speedMetersPerSecond.ofUnit(meters / seconds) / wheelRadius

        if (moduleConstants.useOnboardPID){
            driveMotor.setVelocitySetpoint(
                rawVelocity = velocitySetpoint * moduleConstants.driveGearRatio,
                pidConstants = moduleConstants.velocityPID,
                feedforward = moduleConstants.velocityFF.calculate(velocitySetpoint)
            )
        }else{
            setDriveVoltage(
                (rioVelocityController.calculate(driveAngularVelocity, velocitySetpoint) +
                        moduleConstants.velocityFF.calculate(velocitySetpoint)).coerceIn(getVoltageRange())
            )
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