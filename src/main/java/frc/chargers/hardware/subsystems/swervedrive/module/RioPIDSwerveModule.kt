@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.subsystems.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.DashboardTuner
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.SetpointSupplier
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.subsystems.swervedrive.module.lowlevel.ModuleIO
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import org.littletonrobotics.junction.Logger.recordOutput

/**
 * A swerve module within a [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain]
 * with rio PID control.
 */
public class RioPIDSwerveModule(
    lowLevel: ModuleIO,
    private val controlData: SwerveControlData
): SwerveModule, ModuleIO by lowLevel{
    // ModuleIO by lowLevel makes the lowLevel parameter provide implementation
    // of the ModuleIO interface to the class, reducing boilerplate code


    /**
     * A function that standardizes all angles within the 0 to 360 degree range.
     */
    private fun Angle.standardize(): Angle = this.inputModulus(0.0.degrees..360.degrees)

    /**
     * A function used to calculate the smallest angle delta between 2 angles.
     */
    private fun angleDeltaBetween(angleOne: Angle, angleTwo: Angle): Angle{
        val a1 = angleOne.standardize()
        val a2 = angleTwo.standardize()
        val result = abs(a1-a2)
        return if (360.degrees - result < result){
            360.degrees-result
        }else{
            result
        }
    }

    private val tuner = DashboardTuner()

    private val turnPIDConstants by tuner.pidConstants(
        controlData.anglePID,
        "$logTab/Turning PID Constants"
    )

    private val drivePIDConstants by tuner.pidConstants(
        controlData.velocityPID,
        "$logTab/Driving PID Constants"
    )

    private val velocityController by tuner.refreshWhenTuned{
        SuperPIDController(
            drivePIDConstants,
            getInput = {speed},
            target = AngularVelocity(0.0),
            /**
             * Here, the setpoint supplier is set to the default,
             * with a feedforward that directly corresponds to the input.
             */
            setpointSupplier = SetpointSupplier.Default(
                ffEquation = controlData.velocityFF
            ),
            outputRange = -12.volts..12.volts,
            selfSustain = true
        )
    }


    private val turnController: FeedbackController<Angle, Voltage>
        by tuner.refreshWhenTuned{
            SuperPIDController(
                turnPIDConstants,
                getInput = { direction },
                target = Angle(0.0),
                /**
                 * the [SetpointSupplier] allows the controller
                 * to be a regular PID controller, trapezoid profiled PID controller or
                 * exponential profiled PID controller, depending on user specification.
                 */
                setpointSupplier = controlData.angleSetpointSupplier,
                outputRange = -12.volts..12.volts,
                continuousInputRange = 0.degrees..360.degrees,
                selfSustain = true
            )
        }

    override fun setDirectionalPower(
        power: Double,
        direction: Angle
    ){
        if (angleDeltaBetween(this.direction, direction) > 90.0.degrees){
            setDirection(direction + 180.degrees)
            setPower(-power * cos(turnController.error))
        }else{
            setDirection(direction)
            setPower(power * cos(turnController.error))
        }
    }

    override fun setDirectionalVelocity(
        angularVelocity: AngularVelocity,
        direction: Angle
    ){
        if (angleDeltaBetween(this.direction, direction) > 90.0.degrees){
            setDirection(direction + 180.degrees)
            setVelocity(-angularVelocity * cos(turnController.error))
        }else{
            setDirection(direction)
            setVelocity(angularVelocity * cos(turnController.error))
        }
    }

    // Note: turnSpeed will only be set if the control scheme includes second order kinematics functionality.
    override fun setDirection(direction: Angle){
        turnController.target = direction.standardize()
        // turnVoltage is a setter variable of ModuleIO
        turnVoltage = if ( (turnController.error).within(controlData.modulePrecision) ){
            0.0.volts
        }else{
            turnController.calculateOutput()
        }
        recordOutput("$logTab/withinPrecision", (turnController.error).within(controlData.modulePrecision))
        recordOutput("$logTab/target", turnController.target.siValue)
        recordOutput("$logTab/controllerErrorRad", turnController.error.inUnit(radians))
        recordOutput("$logTab/controllerOutputVolts", turnController.calculateOutput().inUnit(volts))
    }

    private fun setVelocity(velocity: AngularVelocity) {
        velocityController.target = velocity
        // driveVoltage is a setter variable of ModuleIO
        driveVoltage = velocityController.calculateOutput()
    }

    private fun setPower(power: Double) {
        driveVoltage = power * 12.volts
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
        driveVoltage = 0.0.volts
        setDirection(direction)
    }

}