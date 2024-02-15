package frc.robot.hardware.subsystems.pivot

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.inUnit
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
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIO
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

object PivotAngle{
    val AMP: Angle = 30.degrees

    val SOURCE: Angle = 30.degrees

    val SPEAKER: Angle = 0.degrees

    val STOWED: Angle = 0.degrees

    val GROUND_INTAKE_HANDOFF: Angle = 0.degrees
}

class Pivot(
    private val io: PivotIO,
    private val pidConstants: PIDConstants,
    // null indicates no motion profile
    private val motionProfile: AngularMotionProfile? = null,
    private val feedforward: ArmFFEquation = ArmFFEquation(0.0,0.0,0.0),
    private val precision: Precision.Within<AngleDimension> = Precision.Within(0.5.degrees)
): SubsystemBase() {
    private var motionProfileSetpoint = AngularMotionProfileState(io.position)

    @AutoLogOutput
    var hasHitPivotTarget: Boolean = true
        private set


    @AutoLogOutput
    private val mechanismCanvas = Mechanism2d(3.0, 3.0)
    private val pivotVisualizer: MechanismLigament2d

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
        io.setVoltage(0.volts)
    }

    fun setPosition(position: Angle){
        val setpointPosition: Angle
        val feedforward: Voltage

        if (motionProfile != null){
            motionProfileSetpoint = motionProfile.calculate(
                0.02.seconds,
                motionProfileSetpoint,
                AngularMotionProfileState(position)
            )
            setpointPosition = motionProfileSetpoint.position
            feedforward = feedforward(io.position, motionProfileSetpoint.velocity)
        }else{
            setpointPosition = position
            feedforward = 0.volts
        }
        Logger.recordOutput("Shooter/setpoint", setpointPosition.siValue)
        Logger.recordOutput("Shooter/ff", feedforward.siValue)

        hasHitPivotTarget = (setpointPosition - io.position).within(precision)

        if (hasHitPivotTarget){
            io.setVoltage(0.volts)
        }else{
            io.setPosition(
                setpointPosition,
                pidConstants,
                feedforward
            )
        }
    }

    fun setVoltage(voltage: Voltage) = io.setVoltage(voltage)

    fun setAngleCommand(target: Angle): Command =
        buildCommand{
            runOnce(this@Pivot){ setPosition(target) }

            loopUntil({ hasHitPivotTarget }, this@Pivot){
                setPosition(target)
            }

            runOnce(this@Pivot){ setIdle() }
        }

    override fun periodic(){
        pivotVisualizer.angle = io.position.inUnit(degrees)
        Logger.recordOutput(
            "Shooter/PivotPosition3d",
            Pose3d.struct,
            Pose3d(
                Translation3d(0.0, 0.0, 0.0),
                Rotation3d(0.degrees, 0.degrees, 0.degrees)
            )
        )

        setPosition(30.degrees)

        if (DriverStation.isDisabled()){
            setIdle()
        }
    }
}