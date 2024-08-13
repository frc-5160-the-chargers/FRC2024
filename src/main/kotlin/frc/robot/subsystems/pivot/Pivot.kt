package frc.robot.subsystems.pivot

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.feedforward.ArmFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.controls.UnitPIDController
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.utils.Precision
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.Rotation3d


private val STARTING_TRANSLATION_PIVOT_SIM = Translation3d(-0.32, 0.0, 0.72)

// Translation3d(0.0,0.0, 0.0) // -0.25, 0.0, 0.7
@Suppress("unused")
class Pivot(
    private val motor: Motor,
    private val encoderType: PivotEncoderType,

    private val useOnboardPID: Boolean = false,
    private val pidConstants: PIDConstants,
    // null indicates no motion profile
    private val motionProfile: AngularMotionProfile? = null,
    private val feedforward: ArmFFEquation = ArmFFEquation(0.0,0.0,0.0),
    private val precision: Precision.Within<AngleDimension> = Precision.Within(2.degrees),

    private val forwardSoftStop: Angle? = null,
    private val reverseSoftStop: Angle? = null
): SuperSubsystem("Pivot") {
    private var offset by logged(0.degrees, "AngleReadingOffset")

    private val rioController = UnitPIDController<AngleDimension, VoltageDimension>(
        pidConstants,
        outputRange = (-12).volts..12.volts
    )

    val angle: Angle by logged{
        when (encoderType){
            is PivotEncoderType.IntegratedAbsoluteEncoder -> motor.encoder.angularPosition

            is PivotEncoderType.ExternalAbsoluteEncoder -> encoderType.absoluteEncoder.angularPosition

            is PivotEncoderType.IntegratedRelativeEncoder -> (motor.encoder.angularPosition / encoderType.motorGearRatio)
        } - offset
    }

    var atTarget: Boolean by logged(true)
        private set

    init{
        when(encoderType){
            is PivotEncoderType.IntegratedRelativeEncoder -> {
                this.offset = this.angle - encoderType.startingAngle
            }

            is PivotEncoderType.ExternalAbsoluteEncoder -> {
                this.offset = encoderType.offset
            }

            is PivotEncoderType.IntegratedAbsoluteEncoder -> {
                this.offset = encoderType.offset
            }
        }
    }

    private val startingAngle = angle

    private var motionProfileSetpoint = AngularMotionProfileState(angle)

    private fun willExceedSoftStop(movingForward: Boolean): Boolean =
        (forwardSoftStop != null && angle >= forwardSoftStop && movingForward) ||
                (reverseSoftStop != null && angle <= reverseSoftStop && !movingForward)

    private fun resetMotionProfile(){
        if (motionProfile != null){
            motionProfileSetpoint.position = angle
        }
    }

    fun setIdle(){
        resetMotionProfile()
        motor.appliedVoltage = 0.volts
    }

    fun setVoltage(voltage: Voltage){
        resetMotionProfile()
        if (willExceedSoftStop(movingForward = voltage > 0.volts)){
            motor.appliedVoltage = 0.volts
            atTarget = true
            return
        }
        motor.appliedVoltage = voltage
    }

    fun setSpeed(speed: Double) = setVoltage(speed * 12.volts)

    fun setAngle(target: Angle){
        if (willExceedSoftStop(movingForward = this.angle > target)){
            motor.appliedVoltage = 0.volts
            atTarget = true
            return
        }

        val setpointPosition: Angle
        val ffVoltage: Voltage

        if (motionProfile != null){
            motionProfileSetpoint = motionProfile.calculate(
                motionProfileSetpoint,
                AngularMotionProfileState(target),
                0.02.seconds
            )
            setpointPosition = motionProfileSetpoint.position
            ffVoltage = feedforward.calculate(target, motionProfileSetpoint.velocity)
            log("Control/MotionProfileGoal", target)
        }else{
            setpointPosition = target
            ffVoltage = 0.volts
        }
        log("Control/Setpoint", setpointPosition)
        log("Control/FeedForward", ffVoltage)


        val angleDelta = target - this.angle
        atTarget = angleDelta.within(precision)

        if (useOnboardPID || RobotBase.isSimulation()){
            setVoltage(rioController.calculate(angle, target, feedforwardOutput = ffVoltage))
        }else{
            val rawPIDTarget = when (encoderType){
                is PivotEncoderType.IntegratedAbsoluteEncoder -> target
                is PivotEncoderType.ExternalAbsoluteEncoder -> startingAngle + angleDelta * encoderType.motorGearRatio
                is PivotEncoderType.IntegratedRelativeEncoder -> target * encoderType.motorGearRatio
            }
            motor.setPositionSetpoint(rawPIDTarget, pidConstants, feedforward = ffVoltage)
        }
    }

    fun setAngleCommand(target: Angle): Command =
        buildCommand{
            require(this@Pivot)

            runOnce{ setAngle(target) }

            loopUntil({ atTarget }){ setAngle(target) }

            onEnd{ setIdle() }
        }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
            motor.setBrakeMode(false)
        }else{
            motor.setBrakeMode(true)
        }
        log("StatorCurrent", motor.statorCurrent)
        log("VoltageReading", motor.appliedVoltage)
        // logs Mechanism 3d pose for simulation.
        log(
            Pose3d.struct, "Mechanism3dPose",
            Pose3d(
                STARTING_TRANSLATION_PIVOT_SIM,
                Rotation3d(0.degrees, -angle, 0.degrees) // custom overload function that accepts kmeasure quantities
            )
        )
    }
}