package frc.robot.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.wpilibextensions.ratelimit.ScalarRateLimiter
import frc.robot.AIM_TO_TARGET_ENABLED
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.abs
import kotlin.math.hypot


object DriverController: CommandXboxController(0){

    /* Top-Level constants */
    private val aimToTargetPIDConstants =
        if (isReal()) PIDConstants(-0.2,0.0,0.0) else PIDConstants(0.8,0.0,0.0)
    private val translationLimiter: ScalarRateLimiter? = null
    private val rotationLimiter: ScalarRateLimiter? = null






    /* Private implementation variables  */
    private var currentHeading: Angle = 0.0.degrees
    private val aimToAngleController = SuperPIDController(
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


    private val forwardAxis =
        InputAxis{ leftY }
            .applyDeadband(0.1)
            .square()
            .applyMultiplier(0.7)

    private val strafeAxis =
        InputAxis{ leftX }
            .applyDeadband(0.1)
            .square()
            .applyMultiplier(0.7)

    private val rotationAxis =
        InputAxis{ rightX }
            .applyDeadband(0.1)
            .square()
            .applyEquation( Polynomial(0.3,0.0,0.3,0.0) )

    private val turboAxis =
        InputAxis{ leftTriggerAxis }
            .applyDeadband(0.1)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .mapToRange(1.0..2.0)

    private val precisionAxis =
        InputAxis{ leftTriggerAxis }
            .applyDeadband(0.1)
            .mapToRange(1.0..4.0)
            .withModifier{ 1.0 / it }
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }




    fun swerveOutput(robotHeading: Angle? = null): ChassisPowers{
        var forward = forwardAxis()
        var strafe = strafeAxis()
        var rotation = rotationAxis()

        val turbo = turboAxis()
        val precision = precisionAxis()

        if (AIM_TO_TARGET_ENABLED){
            if (robotHeading != null){
                currentHeading = robotHeading
            }

            targetHeading?.let{
                aimToAngleController.target = it
                rotation = aimToAngleController.calculateOutput().siValue
            }

            recordOutput("Drivetrain(Swerve)/Controller/isAiming", targetHeading != null)
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