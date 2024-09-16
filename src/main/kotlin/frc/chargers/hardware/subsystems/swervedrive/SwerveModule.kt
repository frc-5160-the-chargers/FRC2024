package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog.log
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.units.periodToFrequency
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.angle
import kotlin.math.abs

class SwerveModule(
    val name: String,
    private val turnMotor: Motor,
    // turn encoders are optional in sim
    private val turnEncoder: PositionEncoder? = null,
    private val driveMotor: Motor,
    private val moduleConstants: SwerveConstants
) {
    private val startingDirection: Angle? = if (turnEncoder != null) turnEncoder.angularPosition % 360.degrees else null
    private val wheelRadius = moduleConstants.moduleType.wheelDiameter / 2
    private var couplingOffset: Angle = 0.degrees
    private var azimuthProfileState = AngularMotionProfileState(startingDirection ?: turnMotor.encoder.angularPosition)

    val direction: Angle get() = turnMotor.encoder.angularPosition % 360.degrees
    val driveAngularVelocity: AngularVelocity get() = driveMotor.encoder.angularVelocity
    val driveLinearVelocity: Velocity get() = driveAngularVelocity * wheelRadius
    val wheelTravel: Distance get() = driveMotor.encoder.angularPosition * wheelRadius

    init {
        turnMotor.configure(
            inverted = if (moduleConstants.moduleType.turnMotorInverted && RobotBase.isReal()) !turnMotor.inverted else null,
            gearRatio = moduleConstants.moduleType.turnGearRatio,
            startingPosition = startingDirection,
            positionPID = moduleConstants.azimuthPID,
            continuousInput = true,
            positionUpdateRate = periodToFrequency(moduleConstants.odometryUpdateRate)
        )

        driveMotor.configure(
            gearRatio = moduleConstants.moduleType.driveGearRatio,
            startingPosition = 0.degrees,
            velocityPID = moduleConstants.velocityPID,
            positionUpdateRate = periodToFrequency(moduleConstants.odometryUpdateRate)
        )

        log("Data/UsingCouplingRatio", moduleConstants.couplingRatio != null)

        ChargerRobot.runPeriodic {
            if (moduleConstants.couplingRatio != null){
                couplingOffset -= moduleConstants.couplingRatio * (direction % 360.degrees - 180.degrees)
                log("CouplingOffset", couplingOffset)
            }
            log("$name/Direction", direction)
            log("$name/DriveLinearVelocity", driveLinearVelocity)
            log("$name/WheelTravel", wheelTravel)
            log("$name/TurnMotorCurrent", turnMotor.statorCurrent)
            log("$name/DriveMotorCurrent", driveMotor.statorCurrent)
            log("$name/TurnAngularVelocity", turnMotor.encoder.angularVelocity)
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

    fun setDriveVoltage(target: Voltage) {
        driveMotor.appliedVoltage = target
        log("$name/DriveVoltage", target)
    }

    fun setTurnVoltage(target: Voltage) {
        val trueVoltage = if (abs(target) < 0.01.volts){
            0.volts
        }else{
            target
        }
        turnMotor.appliedVoltage = trueVoltage
        log("$name/TurnVoltage", trueVoltage)
    }

    fun setDirection(target: Angle) {
        // utilizes custom absolute value overload for kmeasure quantities
        if (moduleConstants.azimuthPIDTolerance != null &&
            abs(direction - target) < moduleConstants.azimuthPIDTolerance){
            setTurnVoltage(0.volts)
            azimuthProfileState = AngularMotionProfileState(direction)
            return
        }

        val pidTarget: Angle
        val feedforwardV: Voltage

        if (moduleConstants.azimuthMotionProfile != null) {
            val goalState = AngularMotionProfileState(target)

            fun calculateSetpoint(): AngularMotionProfileState =
                moduleConstants.azimuthMotionProfile.calculate(
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

            feedforwardV = moduleConstants.azimuthFF(
                azimuthProfileState.velocity,
                futureSetpoint.velocity
            )
            pidTarget = azimuthProfileState.position
        } else {
            pidTarget = target
            feedforwardV = 0.volts
        }

        // uses pid control to move to the calculated setpoint, to achieve the target goal.
        turnMotor.setPositionSetpoint(pidTarget, feedforwardV)
    }

    fun setDesiredStateOpenLoop(state: SwerveModuleState) {
        val directionAsRotation2d = Rotation2d(direction)
        val optimizedState = SwerveModuleState.optimize(state, directionAsRotation2d)
        optimizedState.speedMetersPerSecond *= abs(
            (optimizedState.angle - directionAsRotation2d).cos
        )
        setDirection(optimizedState.angle.angle)
        setDriveVoltage(
            (optimizedState.speedMetersPerSecond /
                    moduleConstants.driveMotorMaxSpeed.inUnit(meters / seconds) *
                    12.0).coerceIn(getVoltageRange()).ofUnit(volts)
        )
    }

    fun setDesiredStateClosedLoop(state: SwerveModuleState) {
        val directionAsRotation2d = Rotation2d(direction)
        val optimizedState = SwerveModuleState.optimize(state, directionAsRotation2d)
        setDirection(optimizedState.angle.angle)
        val velocitySetpoint = optimizedState.speedMetersPerSecond.ofUnit(meters / seconds) / wheelRadius
        val feedforwardV = moduleConstants.velocityFF(velocitySetpoint)
        driveMotor.setVelocitySetpoint(velocitySetpoint, feedforwardV)
    }

    private val defaultVRange = -12.0..12.0
    private fun getVoltageRange(): ClosedRange<Double>{
        val upperLimit = RobotController.getBatteryVoltage()
        return if (upperLimit < 1.0){
            DriverStation.reportWarning("The battery voltage of the RobotController seems to be extremely low.", true)
            defaultVRange
        }else{
            -upperLimit..upperLimit
        }
    }
}