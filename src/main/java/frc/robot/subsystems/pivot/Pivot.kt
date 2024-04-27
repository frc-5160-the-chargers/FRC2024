package frc.robot.subsystems.pivot

import com.batterystaple.kmeasure.dimensions.AngleDimension
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
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.feedforward.ArmFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.utils.Precision
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.geometry.threedimensional.Rotation3d


private val STARTING_TRANSLATION_PIVOT_SIM = Translation3d(-0.32, 0.0, 0.72)

// Translation3d(0.0,0.0, 0.0) // -0.25, 0.0, 0.7
@Suppress("unused")
class Pivot(
    private val motor: MotorizedComponent,
    private val gearRatio: Double,
    private val encoderType: PivotEncoderType,

    private val pidConstants: PIDConstants,
    // null indicates no motion profile
    private val motionProfile: AngularMotionProfile? = null,
    private val feedforward: ArmFFEquation = ArmFFEquation(0.0,0.0,0.0),
    private val precision: Precision.Within<AngleDimension> = Precision.Within(2.degrees),

    private val forwardSoftStop: Angle? = null,
    private val reverseSoftStop: Angle? = null
): SuperSubsystem("Pivot") {
    private var offset by logged(0.degrees, "AngleReadingOffset")

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

    fun setAngle(angle: Angle){
        if (willExceedSoftStop(movingForward = angle > angle)){
            motor.appliedVoltage = 0.volts
            atTarget = true
            return
        }

        val setpointPosition: Angle
        val feedforward: Voltage

        if (motionProfile != null){
            motionProfileSetpoint = motionProfile.calculate(
                0.02.seconds,
                motionProfileSetpoint,
                AngularMotionProfileState(angle)
            )
            setpointPosition = motionProfileSetpoint.position
            feedforward = feedforward(angle, motionProfileSetpoint.velocity)
            log("Control/MotionProfileGoal", angle)
        }else{
            setpointPosition = angle
            feedforward = 0.volts
        }
        log("Control/Setpoint", setpointPosition)
        log("Control/FeedForward", feedforward)

        atTarget = (angle - this.angle).within(precision)

        motor.setPositionSetpoint(
            rawPosition = setpointPosition * gearRatio,
            pidConstants,
            feedforward = feedforward
        )
    }

    fun setAngleCommand(target: Angle): Command =
        buildCommand{
            runOnce(this@Pivot){ setAngle(target) }

            loopUntil({ atTarget }, this@Pivot){
                setAngle(target)
            }

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