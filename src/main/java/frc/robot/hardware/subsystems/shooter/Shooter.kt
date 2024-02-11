@file:Suppress("unused", "MemberVisibilityCanBePrivate")
package frc.robot.hardware.subsystems.shooter

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
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
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIO
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger.recordOutput



object PivotAngle{
    val AMP: Angle = 0.degrees

    val SOURCE: Angle = 0.degrees

    val SPEAKER: Angle = 0.degrees

    val STOWED: Angle = 0.degrees

    val GROUND_INTAKE_HANDOFF: Angle = 0.degrees
}


class Shooter(
    private val io: ShooterIO,
    private val pivotPID: PIDConstants,
    // null indicates no motion profile
    private val pivotProfile: AngularMotionProfile? = null,
    private val pivotFF: ArmFFEquation = ArmFFEquation(0.0,0.0,0.0),
    private val pivotPrecision: Precision.Within<AngleDimension> = Precision.Within(0.5.degrees)
): SubsystemBase() {
    private var motionProfileSetpoint = AngularMotionProfileState(io.pivotPosition)

    @get:AutoLogOutput
    val hasHitPivotTarget: Boolean get() {
        return io.pivotPosition.within(pivotPrecision)
    }

    val hasGamepiece: Boolean get() = io.hasGamepiece

    val hasBeamBreakSensor: Boolean get() = io.hasBeamBreakSensor


    @AutoLogOutput
    val mechanismCanvas = Mechanism2d(3.0, 3.0)
    val pivotVisualizer: MechanismLigament2d

    init{
        val root = mechanismCanvas.getRoot("PivotingShooter", 2.0, 0.0)

        val staticJoint = root.append(
            MechanismLigament2d(
                "staticJoint",
                0.5,
                90.0
            )
        )

        pivotVisualizer = staticJoint.append(
            MechanismLigament2d(
                "pivot",
                0.2,
                180.0,
                3.0,
                Color8Bit(Color.kRed)
            )
        )
    }







    fun setIdle(){
        io.setPivotVoltage(0.volts)
        io.setVoltage(0.volts)
    }

    fun setPivotPosition(position: Angle){
        val setpointPosition: Angle
        val feedforward: Voltage

        if (pivotProfile != null){
            motionProfileSetpoint = pivotProfile.calculate(
                0.02.seconds,
                motionProfileSetpoint,
                AngularMotionProfileState(position)
            )
            setpointPosition = motionProfileSetpoint.position
            feedforward = pivotFF(io.pivotPosition, motionProfileSetpoint.velocity)
        }else{
            setpointPosition = position
            feedforward = 0.volts
        }

        if (hasHitPivotTarget){
            io.setPivotVoltage(0.volts)
        }else{
            io.setPivotPosition(
                setpointPosition,
                pivotPID,
                feedforward
            )
        }
    }
    
    fun setPivotVoltage(voltage: Voltage) = io.setPivotVoltage(voltage)

    fun setPivotSpeed(percentOut: Double) = io.setPivotVoltage(percentOut * 11.volts)




    fun intake(percentOut: Double){
        intake(percentOut * 12.volts)
    }

    fun intake(voltage: Voltage){
        if (io.hasGamepiece && io.hasBeamBreakSensor){
            io.setVoltage(0.volts)
        }else{
            io.setVoltage(-voltage)
        }
    }


    fun outtake(percentOut: Double){
        outtake(percentOut * 12.volts)
    }

    fun outtake(voltage: Voltage){
        require(voltage >= 0.volts){ "Applied voltage must be > 0.volts. To intake, call intake(voltage) or intake(speed) instead." }
        io.setVoltage(voltage)
    }



    fun setAngleCommand(target: Angle): Command =
        buildCommand{
            runOnce(this@Shooter){ setPivotPosition(target) }

            loopUntil({ hasHitPivotTarget }, this@Shooter){
                setPivotPosition(target)
            }

            runOnce(this@Shooter){ setPivotVoltage(0.volts) }
        }

    fun shootInSpeaker(
        percentOut: Double,
        time: Time = 0.3.seconds,
        stowOnEnd: Boolean = false
    ): Command = buildCommand {
        +setAngleCommand(PivotAngle.SPEAKER)

        loopFor(time, this@Shooter) {
            outtake(percentOut)
        }

        runOnce {
            setIdle()
        }

        if (stowOnEnd) {
            +setAngleCommand(PivotAngle.STOWED)
        }
    }

    fun shootInAmp(
        percentOut: Double,
        time: Time = 0.3.seconds,
        stowOnEnd: Boolean = false
    ): Command = buildCommand {
        +setAngleCommand(PivotAngle.AMP)

        loopFor(time, this@Shooter) {
            outtake(percentOut)
        }

        runOnce {
            setIdle()
        }

        if (stowOnEnd) {
            +setAngleCommand(PivotAngle.STOWED)
        }
    }








    override fun periodic(){
        pivotVisualizer.angle = io.pivotPosition.inUnit(degrees)
        recordOutput(
            "Shooter/PivotPosition3d",
            Pose3d.struct,
            Pose3d(
                Translation3d(0.0,0.0,0.0),
                Rotation3d(0.degrees, 0.degrees, 0.degrees)
            )
        )

        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}
