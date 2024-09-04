package frc.robot.rigatoni.subsystems.pivot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.feedforward.ArmFFEquation
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.encoders.ChargerDutyCycleEncoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.wpilibextensions.Rotation3d

private const val PIVOT_MOTOR_ID = 9
private const val PIVOT_ENCODER_ID = 0

private val PIVOT_SIM_STARTING_TRANSLATION = Translation3d(-0.32, 0.0, 0.72)
private val FORWARD_LIMIT: Angle = 1.636.radians
private val REVERSE_LIMIT: Angle = (-1.8).radians
private val PID_TOLERANCE = 1.3.degrees
private val ABSOLUTE_ENCODER_OFFSET = (-0.23).radians

class Pivot: SuperSubsystem("Pivot") {
    private val motor: Motor
    private val absoluteEncoder: PositionEncoder?

    init {
        if (isSimulation()){
            motor = MotorSim(DCMotor.getNEO(1), moi = 0.004.kilo.grams * (meters * meters))
            absoluteEncoder = null
        }else{
            motor = ChargerSparkMax(PIVOT_MOTOR_ID)
            absoluteEncoder = ChargerDutyCycleEncoder(PIVOT_ENCODER_ID) + ABSOLUTE_ENCODER_OFFSET
        }
    }

    private val startingAngle = absoluteEncoder?.angularPosition ?: PivotAngle.STARTING

    init {
        motor.configure(
            optimizeUpdateRate = true,
            statorCurrentLimit = 35.amps,
            startingPosition = startingAngle,
            gearRatio = 96.0,
            positionPID = PIDConstants(7.0,0.0,0.001)
        )
    }

    private val motionProfile: AngularMotionProfile? = AngularTrapezoidProfile(
        maxVelocity = AngularVelocity(8.0),
        maxAcceleration = AngularAcceleration(10.0)
    )
    private var motionProfileSetpoint = AngularMotionProfileState(startingAngle)
    private val feedforward = ArmFFEquation(0.0, 0.0, 0.0)

    val angle: Angle by logged { motor.encoder.angularPosition }

    var atTarget: Boolean by logged(true)
        private set

    private fun willExceedSoftStop(movingForward: Boolean): Boolean =
        (angle >= FORWARD_LIMIT && movingForward) || (angle <= REVERSE_LIMIT && !movingForward)

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

        val pidTarget: Angle
        val ffVoltage: Voltage
        if (motionProfile != null){
            motionProfileSetpoint = motionProfile.calculate(
                motionProfileSetpoint,
                AngularMotionProfileState(target),
                0.02.seconds
            )
            pidTarget = motionProfileSetpoint.position
            ffVoltage = feedforward.calculate(target, motionProfileSetpoint.velocity)
            log("Control/MotionProfileGoal", target)
        }else{
            pidTarget = target
            ffVoltage = 0.volts
        }
        motor.setPositionSetpoint(pidTarget, ffVoltage)

        log("Control/Setpoint", pidTarget)
        log("Control/FeedForward", ffVoltage)
        atTarget = (target - this.angle) < PID_TOLERANCE
    }

    fun setAngleCommand(target: Angle): Command =
        buildCommand("SetAngleCommand") {
            require(this@Pivot)

            runOnce{ setAngle(target) }

            loopUntil({ atTarget }){ setAngle(target) }

            onEnd{ setIdle() }
        }

    override fun periodic(){
        if (DriverStation.isDisabled()){
            setIdle()
            motor.configure(brakeWhenIdle = false)
        }else{
            motor.configure(brakeWhenIdle = true)
        }
        log("StatorCurrent", motor.statorCurrent)
        log("VoltageReading", motor.appliedVoltage)
        // logs Mechanism 3d pose for simulation.
        log(
            Pose3d.struct, "Mechanism3dPose",
            Pose3d(
                PIVOT_SIM_STARTING_TRANSLATION,
                Rotation3d(0.degrees, -angle, 0.degrees) // custom overload function that accepts kmeasure quantities
            )
        )
    }
}