package frc.robot.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.inputdevices.ChargerController
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.wpilibextensions.ratelimit.ScalarRateLimiter
import frc.robot.AIM_TO_TARGET_ENABLED
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.abs
import kotlin.math.hypot


object DriverController: ChargerController(port = 0, deadband = 0.1){

    /* Top-Level constants */
    private val aimToTargetPIDConstants = if (isReal()) PIDConstants(-0.2,0.0,0.0) else PIDConstants(0.8,0.0,0.0)

    // 0.2x^3 + 0.5x
    private val driveMultiplierFunction = Polynomial(0.2,0.0,0.5,0.0)
    // 0.3x^3 + 0.3x
    private val rotationMultiplierFunction = Polynomial(0.3,0.0,0.3,0.0)

    private val translationLimiter: ScalarRateLimiter? = null
    private val rotationLimiter: ScalarRateLimiter? = null

    private val turboModeMultiplier = 1.0..2.0
    private val precisionModeDivider = 1.0..4.0





    /* Private implementation variables  */
    private var currentHeading: Angle = 0.0.degrees
    private var aimToAngleController = SuperPIDController(
        aimToTargetPIDConstants,
        getInput = {currentHeading.inputModulus(0.0.degrees..360.degrees)},
        outputRange = Scalar(-0.7)..Scalar(0.7),
        target = 0.0.degrees,
        continuousInputRange = 0.0.degrees..360.degrees
    )
    private var previousForward = 0.0
    private var previousStrafe = 0.0
    private var previousRotation = 0.0







    /* Public API */
    var targetHeading: Angle? = null
    val pointNorthButton: Trigger = y()
    val pointSouthButton: Trigger = a()
    val pointEastButton: Trigger = x()
    val pointWestButton: Trigger = b()
    val headingZeroButton: Trigger = if (isReal()) back() else Trigger{false}
    val poseZeroButton: Trigger = if (isReal()) start() else Trigger{false}

    fun swerveOutput(robotHeading: Angle? = null): ChassisPowers{
        var forward = driveMultiplierFunction( leftY.withScaledDeadband() )
        var strafe = driveMultiplierFunction( leftX.withScaledDeadband() )
        var rotation = rotationMultiplierFunction( rightX.withScaledDeadband() )


        var turbo = abs(leftTriggerAxis).mapTriggerValue(turboModeMultiplier)
        var precision = 1 / abs(rightTriggerAxis).mapTriggerValue(precisionModeDivider)

        if (turbo < 1.0 || precision.isInfinite() || precision.isNaN()) turbo = 1.0
        if (precision.isInfinite() || precision.isNaN() || precision > 1.0 || precision == 0.0) precision = 1.0

        if (AIM_TO_TARGET_ENABLED){
            if (robotHeading != null){
                currentHeading = robotHeading
            }

            targetHeading?.let{
                aimToAngleController.target = it
                rotation = aimToAngleController.calculateOutput().siValue
                println("Aiming!")
            }
        }


        val magnitude = hypot(forward,strafe)
        val limitedMagnitude = translationLimiter?.calculate(magnitude) ?: magnitude

        if (abs(forward) > abs(previousForward)) forward *= limitedMagnitude / magnitude
        if (abs(strafe) > abs(previousStrafe)) strafe *= limitedMagnitude / magnitude
        if (rotationLimiter != null && abs(rotation) > abs(previousRotation)) {
            rotation = rotationLimiter.calculate(rotation)
        }else{
            rotationLimiter?.reset(rotation)
        }

        previousForward = forward
        previousStrafe = strafe
        previousRotation = rotation

        recordOutput("Drivetrain(Swerve)/rotation output", rotation)
        recordOutput("Drivetrain(Swerve)/xPower", forward)
        recordOutput("Drivetrain(Swerve)/yPower", strafe)

        return ChassisPowers(
            xPower = forward * turbo * precision,
            yPower = strafe * turbo * precision,
            rotationPower = rotation * precision
        )
    }



}