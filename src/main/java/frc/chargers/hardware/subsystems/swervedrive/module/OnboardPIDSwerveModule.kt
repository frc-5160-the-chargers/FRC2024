@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.drivetrain.DEFAULT_GEAR_RATIO
import frc.chargers.constants.drivetrain.SwerveControlData
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.subsystems.swervedrive.module.lowlevel.ModuleIO
import frc.chargers.hardware.subsystems.swervedrive.module.lowlevel.ModuleIOReal
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import kotlin.math.abs

/**
 * A [SwerveModule] that uses onboard PID.
 */
public class OnboardPIDSwerveModule private constructor( // Do not use this constructor
    lowLevel: ModuleIOReal,
    private val controlData: SwerveControlData,
    private val turnMotor: SmartEncoderMotorController,
    private val turnEncoder: PositionEncoder,
    private val driveMotor: SmartEncoderMotorController
): ModuleIO by lowLevel, SwerveModule {
    // ModuleIO by lowLevel makes the lowLevel parameter provide implementation
    // of the ModuleIO interface to the class, reducing boilerplate code

    public constructor(
        logInputs: LoggableInputsProvider,
        controlData: SwerveControlData,
        turnMotor: SmartEncoderMotorController,
        turnEncoder: PositionEncoder,
        driveMotor: SmartEncoderMotorController,
        driveGearRatio: Double = DEFAULT_GEAR_RATIO,
        turnGearRatio: Double = DEFAULT_GEAR_RATIO,
    ): this(
        ModuleIOReal(
            logInputs, turnMotor, turnEncoder, driveMotor, driveGearRatio, turnGearRatio
        ),
        controlData, turnMotor, turnEncoder, driveMotor
    )

    private val tuner = DashboardTuner()

    private val turnPIDConstants by tuner.pidConstants(
        controlData.anglePID,
        "$logTab/Turning PID Constants"
    )

    private val drivePIDConstants by tuner.pidConstants(
        controlData.velocityPID,
        "$logTab/Driving PID Constants"
    )

    override fun setDirectionalPower(power: Double, direction: Angle) {
        val modState = optimizeOnboardPIDState(
            SwerveModuleState(power,direction.asRotation2d()),
            turnEncoder.angularPosition.asRotation2d()
        )
        driveVoltage = (modState.speedMetersPerSecond * 12.volts).coerceIn(-12.volts..12.volts)
        setDirection(modState.angle.asAngle())
    }

    override fun setDirectionalVelocity(angularVelocity: AngularVelocity, direction: Angle) {
        val modState = optimizeOnboardPIDState(
            SwerveModuleState(angularVelocity.siValue, direction.asRotation2d()),
            turnEncoder.angularPosition.asRotation2d()
        )
        driveMotor.setAngularVelocity(
            AngularVelocity(modState.speedMetersPerSecond),
            drivePIDConstants,
            controlData.velocityFF
        )
        setDirection(modState.angle.asAngle())
    }

    override fun getModuleState(wheelRadius: Length): SwerveModuleState =
        SwerveModuleState(
            speed.inUnit(radians / seconds) * wheelRadius.inUnit(meters),
            direction.asRotation2d()
        )

    override fun getModulePosition(wheelRadius: Length): SwerveModulePosition =
        SwerveModulePosition(
            wheelTravel.inUnit(radians) * wheelRadius.inUnit(meters),
            direction.asRotation2d()
        )

    override fun halt() {
        setDirectionalPower(0.0,direction)
    }

    override fun setDirection(direction: Angle) {
        val turnSetpoint =
            controlData.angleSetpointSupplier.calculateSetpoint(direction)

        turnMotor.setAngularPosition(
            turnSetpoint.value,
            turnPIDConstants,
            continuousWrap = true,
            extraVoltage = turnSetpoint.feedforwardOutput,
            turnEncoder = turnEncoder
        )
    }


}

/**
 * Minimize the change in heading the desired swerve module state would require by potentially
 * reversing the direction the wheel spins. Customized from WPILib's version to include placing
 * in appropriate scope for CTRE onboard control.
 *
 * Credits: 364 base falcon swerve
 *
 * @param desiredState The desired state.
 * @param currentAngle The current module angle.
 */
private fun optimizeOnboardPIDState(desiredState: SwerveModuleState, currentAngle: Rotation2d): SwerveModuleState {
    var targetAngle = placeInAppropriate0To360Scope(currentAngle.degrees, desiredState.angle.degrees)
    var targetSpeed = desiredState.speedMetersPerSecond
    val delta = targetAngle - currentAngle.degrees
    if (abs(delta) > 90) {
        targetSpeed = -targetSpeed
        targetAngle =
            if (delta > 90) 180.let { targetAngle -= it; targetAngle } else 180.let { targetAngle += it; targetAngle }
    }
    return SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle))
}

/**
 * Credits: FRC 364 base falcon swerve
 *
 * @param scopeReference Current Angle
 * @param newAngle Target Angle
 * @return Closest angle within scope
 */
@Suppress("NAME_SHADOWING")
private fun placeInAppropriate0To360Scope(scopeReference: Double, newAngle: Double): Double {
    var newAngle = newAngle
    val lowerBound: Double
    val upperBound: Double
    val lowerOffset = scopeReference % 360
    if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset
        upperBound = scopeReference + (360 - lowerOffset)
    } else {
        upperBound = scopeReference - lowerOffset
        lowerBound = scopeReference - (360 + lowerOffset)
    }
    while (newAngle < lowerBound) {
        newAngle += 360.0
    }
    while (newAngle > upperBound) {
        newAngle -= 360.0
    }
    if (newAngle - scopeReference > 180) {
        newAngle -= 360.0
    } else if (newAngle - scopeReference < -180) {
        newAngle += 360.0
    }
    return newAngle
}



