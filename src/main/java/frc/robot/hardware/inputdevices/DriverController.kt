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
import frc.chargers.utils.math.equations.epsilonEquals
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.constants.AIM_TO_TARGET_ENABLED
import frc.robot.constants.DRIVER_CONTROLLER_PORT
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.math.pow


object DriverController: CommandXboxController(DRIVER_CONTROLLER_PORT){

    /* Top-Level constants */
    private val aimToTargetPIDConstants =
        if (isReal()) PIDConstants(0.2,0.0,0.0) else PIDConstants(0.8,0.0,0.0)
    private const val DEFAULT_DEADBAND = 0.1


    /* Public API */
    /**
     * The target aim to angle heading of the
     */
    var targetHeading: Angle? = null

    val pointNorthButton: Trigger = y()
    val pointSouthButton: Trigger = a()
    val pointEastButton: Trigger = x()
    val pointWestButton: Trigger = b()
    val headingZeroButton: Trigger = if (isReal()) back() else Trigger{false}
    val poseZeroButton: Trigger = if (isReal()) start() else Trigger{false}


    /* Private implementation */
    private var currentHeading: Angle = 0.0.degrees
    private val aimToAngleController = SuperPIDController(
        aimToTargetPIDConstants,
        getInput = {currentHeading.inputModulus(0.0.degrees..360.degrees)},
        outputRange = Scalar(-0.7)..Scalar(0.7),
        target = 0.0.degrees,
        continuousInputRange = 0.0.degrees..360.degrees
    )

    private fun InputAxis.applyDefaults(): InputAxis =
        this.applyDeadband(DEFAULT_DEADBAND)


    private val driveScalar =
        InputAxis{ leftX.pow(2) + leftY.pow(2) }
            .square()
            .applyMultiplier(0.8)

    private fun getScaleRate(): Double{
        val baseValue = driveScalar.getBaseValue()
        val scaledValue = driveScalar()

        return if (baseValue epsilonEquals 0.0){
            1.0
        }else {
            scaledValue / baseValue
        }.also{ recordOutput("Drivetrain(Swerve)/scaleRate", it) }
    }




    private val forwardAxis =
        InputAxis{ leftY }
            .applyDefaults()
            .applyMultiplier(-1.0)
            .withModifier{ it * getScaleRate() }

    private val strafeAxis =
        InputAxis{ leftX }
            .applyMultiplier(-1.0)
            .withModifier{ it * getScaleRate() }

    private val rotationAxis =
        InputAxis{ rightX }
            .applyDefaults()
            .square()
            .applyEquation( Polynomial(0.1,0.0,0.5,0.0) )

    private val turboAxis =
        InputAxis{ leftTriggerAxis }
            .applyDefaults()
            .mapToRange(1.0..2.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }

    private val precisionAxis =
        InputAxis{ leftTriggerAxis }
            .applyDefaults()
            .mapToRange(1.0..4.0)
            .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
            .withModifier{ 1.0 / it }



    fun swerveOutput(robotHeading: Angle? = null): ChassisPowers{
        val forward = forwardAxis()
        val strafe = strafeAxis()
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

        recordOutput("Drivetrain(Swerve)/xPower", forward * turbo * precision)
        recordOutput("Drivetrain(Swerve)/yPower", strafe * turbo * precision)
        recordOutput("Drivetrain(Swerve)/rotation output", rotation * precision)

        return ChassisPowers(
            xPower = forward * turbo * precision,
            yPower = strafe * turbo * precision,
            rotationPower = rotation * precision
        )
    }



}