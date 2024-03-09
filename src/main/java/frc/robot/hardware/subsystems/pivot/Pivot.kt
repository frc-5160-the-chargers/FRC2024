package frc.robot.hardware.subsystems.pivot

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.feedforward.ArmFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.utils.within
import frc.chargers.wpilibextensions.geometry.threedimensional.Rotation3d
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIO
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger


private val STARTING_TRANSLATION_PIVOT_SIM = Translation3d(0.325, 0.0, 0.75)

class Pivot(
    val io: PivotIO,
    private val pidConstants: PIDConstants,
    // null indicates no motion profile
    private val motionProfile: AngularMotionProfile? = null,
    private val feedforward: ArmFFEquation = ArmFFEquation(0.0,0.0,0.0),
    private val precision: Precision.Within<AngleDimension> = Precision.Within(1.5.degrees),
    private val forwardSoftStop: Angle? = null,
    private val reverseSoftStop: Angle? = null
): SubsystemBase() {
    /* Private Members */
    private var motionProfileSetpoint = AngularMotionProfileState(io.angle)

    private fun willExceedSoftStop(movingForward: Boolean): Boolean =
        (forwardSoftStop != null && io.angle >= forwardSoftStop && movingForward) ||
        (reverseSoftStop != null && io.angle <= reverseSoftStop && !movingForward)

    init{
        io.zeroAngle(PivotAngle.STOWED) // assumes pivot is stowed at the start
    }

    /* Public API */
    @AutoLogOutput
    var atTarget: Boolean = true
        private set

    // regular pose3d is the only option because UnitPose3d does not support automatic logging
    @get:AutoLogOutput
    val mechanism3dPose: Pose3d
        get() = Pose3d(
            STARTING_TRANSLATION_PIVOT_SIM,
            Rotation3d(0.degrees, io.angle, 0.degrees) // custom overload function that accepts kmeasure quantities
        )

    val angle: Angle
        get() = io.angle

    fun setIdle(){
        io.setVoltage(0.volts)
    }

    fun setVoltage(voltage: Voltage){
        if (willExceedSoftStop(movingForward = voltage > 0.volts)){
            setIdle()
            atTarget = true
            return
        }
        io.setVoltage(voltage)
    }

    fun setSpeed(speed: Double) = setVoltage(speed * 12.volts)

    fun setAngle(angle: Angle){
        if (willExceedSoftStop(movingForward = angle > io.angle)){
            setIdle()
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
            feedforward = feedforward(io.angle, motionProfileSetpoint.velocity)
        }else{
            setpointPosition = angle
            feedforward = 0.volts
        }
        Logger.recordOutput("Pivot/setpoint", setpointPosition.siValue)
        Logger.recordOutput("Pivot/ff", feedforward.siValue)

        atTarget = (angle - io.angle).within(precision)

        io.setAngleSetpoint(
            setpointPosition,
            pidConstants,
            feedforward
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
        }
    }
}