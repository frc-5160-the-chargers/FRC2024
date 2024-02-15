@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.subsystems.swervedrive.module

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.DashboardTuner
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.controls.FeedbackController
import frc.chargers.controls.motionprofiling.MotionProfileState
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.controls.pid.SuperProfiledPIDController
import frc.chargers.framework.ChargerRobot
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
    io: ModuleIO,
    private val controlData: SwerveControlData
): SwerveModule, ModuleIO by io{
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
        controlData.azimuthControl.pidConstants,
        "$logTab/Turning PID Constants"
    )

    private val drivePIDConstants by tuner.pidConstants(
        controlData.velocityPID,
        "$logTab/Driving PID Constants"
    )

    private val velocityController by tuner.refreshWhenTuned{
        SuperPIDController(
            drivePIDConstants,
            controlData.velocityFF,
            getInput = {speed},
            target = AngularVelocity(0.0),
            outputRange = -12.volts..12.volts,
            selfSustain = true
        )
    }


    private val turnController: FeedbackController<Angle, Voltage>
        by tuner.refreshWhenTuned{
            when (controlData.azimuthControl){
                is SwerveAzimuthControl.ProfiledPID -> SuperProfiledPIDController(
                    turnPIDConstants,
                    controlData.azimuthControl.motionProfile,
                    getInput = { direction },
                    targetState = MotionProfileState(Angle(0.0)),
                    outputRange = -12.volts..12.volts,
                    continuousInputRange = 0.degrees..360.degrees,
                    selfSustain = true,
                    feedforward = ff@{ // has the context of the SuperProfiledPIDController class; thus, setpoint and targetState are pid controller properties
                        // fetches the current setpoint velocity; this is not the goal(targetState) of the controller
                        val currentVelocitySetpoint: AngularVelocity = setpoint.velocity

                        // calculates the profile state 1 loop ahead, and gets the velocity from it
                        val nextVelocitySetpoint: AngularVelocity =
                            controlData.azimuthControl.motionProfile
                                .calculate(ChargerRobot.LOOP_PERIOD, setpoint, targetState)
                                .velocity

                        // uses linear plant inversion to calculate optimal feedforward output
                        return@ff controlData.azimuthControl.ffEquation.calculatePlantInversion(
                            currentVelocitySetpoint,
                            nextVelocitySetpoint
                        )
                    },
                )

                is SwerveAzimuthControl.PID -> SuperPIDController(
                    turnPIDConstants,
                    getInput = { direction },
                    target = Angle(0.0),
                    outputRange = -12.volts..12.volts,
                    continuousInputRange = 0.degrees..360.degrees,
                    selfSustain = true
                )
            }
        }

    override fun setDirectionalPower(
        power: Double,
        direction: Angle
    ){
        val optimizedPower: Double

        if (angleDeltaBetween(this.direction, direction) > 90.0.degrees){
            setDirection(direction + 180.degrees)
            optimizedPower = -power * cos(turnController.error)
        }else{
            setDirection(direction)
            optimizedPower = power * cos(turnController.error)
        }

        driveVoltage = if (optimizedPower > 0.0){
            (optimizedPower * 12.volts) + controlData.velocityFF.kS
        }else if (optimizedPower < 0.0){
            (optimizedPower * 12.volts) - controlData.velocityFF.kS
        }else{
            0.volts
        }
    }

    override fun setDirectionalVelocity(
        angularVelocity: AngularVelocity,
        direction: Angle
    ){
        val optimizedVelocity: AngularVelocity

        if (angleDeltaBetween(this.direction, direction) > 90.0.degrees){
            setDirection(direction + 180.degrees)
            optimizedVelocity = -angularVelocity * cos(turnController.error)
        }else{
            setDirection(direction)
            optimizedVelocity = angularVelocity * cos(turnController.error)
        }

        velocityController.target = optimizedVelocity
        // driveVoltage is a setter variable of ModuleIO
        driveVoltage = velocityController.calculateOutput()
    }

    // Note: turnSpeed will only be set if the control scheme includes second order kinematics functionality.
    override fun setDirection(direction: Angle){
        turnController.target = direction.standardize()
        // turnVoltage is a setter variable of ModuleIO
        turnVoltage = if ( (turnController.error).within(controlData.azimuthControl.precision) ){
            0.0.volts
        }else{
            turnController.calculateOutput()
        }
        recordOutput("$logTab/withinPrecision", (turnController.error).within(controlData.azimuthControl.precision))
        recordOutput("$logTab/target", turnController.target.siValue)
        recordOutput("$logTab/controllerErrorRad", turnController.error.inUnit(radians))
        recordOutput("$logTab/controllerOutputVolts", turnController.calculateOutput().inUnit(volts))
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
}