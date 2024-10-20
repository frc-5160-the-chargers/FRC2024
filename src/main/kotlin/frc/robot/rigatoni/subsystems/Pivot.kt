package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.framework.HorseLog.log
import frc.chargers.framework.logged
import frc.chargers.framework.tunable
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.encoders.ChargerDutyCycleEncoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.wpilibextensions.Rotation3d
import kcommand.InstantCommand

// Angle values: - is outward, + is inward
// voltage: + is outward, - is inward
private const val PIVOT_MOTOR_ID = 9
private const val PIVOT_ENCODER_ID = 0

private val PID_TOLERANCE = 2.degrees
private val ABSOLUTE_ENCODER_OFFSET = -1.404.radians
private val PIVOT_SIM_STARTING_TRANSLATION = Translation3d(-0.32, 0.0, 0.72)
private val PIVOT_MOI = 0.008.kilo.grams * (meters * meters)

// upright = 0 rad
object PivotAngle {
    val AMP = -0.72.radians
    val SOURCE = 0.radians
    val GROUND_INTAKE_HANDOFF = 0.9.radians
    val STOWED = GROUND_INTAKE_HANDOFF // same as of now
    val SPEAKER = 0.77.radians
    val STARTING = 1.15.radians // used to zero the encoder if there is no absolute encoder(we have one)
}

class Pivot(disable: Boolean = false): SubsystemBase() {
    private val forwardLimit by tunable(-1.636.radians)
    private val reverseLimit by tunable(1.15.radians)
    private val pivotPIDConstants by tunable(PIDConstants(0.7, 0.0, 0.001))
        .onChange { motor.configure(positionPID = it) }

    private val motor: Motor
    private val absoluteEncoder: PositionEncoder?

    init {
        if (isSimulation() || disable){
            motor = MotorSim(DCMotor.getNEO(1), PIVOT_MOI)
            absoluteEncoder = null
        }else{
            motor = ChargerSparkMax(PIVOT_MOTOR_ID, faultLogName = "PivotMotor")
                .configure(inverted = true)
            absoluteEncoder = ChargerDutyCycleEncoder(PIVOT_ENCODER_ID) + ABSOLUTE_ENCODER_OFFSET
        }
    }

    private val startingAngle = absoluteEncoder?.angularPosition ?: PivotAngle.STARTING

    init {
        motor.configure(
            optimizeUpdateRate = true,
            statorCurrentLimit = 35.amps,
            currentPosition = startingAngle,
            gearRatio = 96.0,
            positionPID = pivotPIDConstants
        )
        Trigger(DriverStation::isDisabled)
            .onTrue(InstantCommand { setIdle(); motor.configure(brakeWhenIdle = false) }.ignoringDisable(true))
            .onFalse(InstantCommand { motor.configure(brakeWhenIdle = true) })
    }

    private val motionProfile: AngularMotionProfile? = AngularTrapezoidProfile(
        maxVelocity = AngularVelocity(8.0),
        maxAcceleration = AngularAcceleration(10.0)
    )
    private var motionProfileSetpoint = AngularMotionProfileState(startingAngle)
    private val feedforward = AngularMotorFeedforward(0.0, 0.0, 0.0)

    val angle: Angle by logged { motor.encoder.angularPosition }

    var atTarget: Boolean by logged(true)
        private set

    // note: forward is negative, so we do <= forward limit
    private fun willExceedSoftStop(movingForward: Boolean): Boolean =
        (angle <= forwardLimit && movingForward) || (angle >= reverseLimit && !movingForward)

    private fun resetMotionProfile(){
        if (motionProfile != null) motionProfileSetpoint.position = angle
    }

    fun setIdle(){
        resetMotionProfile()
        motor.voltageOut = 0.volts
    }

    fun setVoltage(voltage: Voltage){
        resetMotionProfile()
        if (willExceedSoftStop(movingForward = voltage < 0.volts)){
            setIdle()
            atTarget = true
            return
        }
        motor.voltageOut = voltage
    }

    fun setSpeed(speed: Double) = setVoltage(speed * 12.volts)

    fun setAngle(target: Angle) {
        if (willExceedSoftStop(movingForward = this.angle < target)){
            setIdle()
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
            ffVoltage = feedforward(motionProfileSetpoint.velocity)
            log("Pivot/Control/MotionProfileGoal", target)
        }else{
            pidTarget = target
            ffVoltage = 0.volts
        }
        motor.setPositionSetpoint(pidTarget, ffVoltage)

        log("Pivot/Control/Setpoint", pidTarget)
        log("Pivot/Control/FeedForward", ffVoltage)
        atTarget = abs(target - this.angle) < PID_TOLERANCE
    }

    override fun periodic(){
        log("Pivot/StatorCurrent", motor.statorCurrent)
        log("Pivot/VoltageReading", motor.voltageOut)
        if (isSimulation()) {
            log( // logs Mechanism 3d pose for simulation.
                "Pivot/Mechanism3dPose",
                // custom overload function that accepts kmeasure quantities
                Pose3d(PIVOT_SIM_STARTING_TRANSLATION, Rotation3d(0.degrees, -angle, 0.degrees))
            )
        }
    }
}