package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.encoders.ChargerDutyCycleEncoder
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.wpilibextensions.Rotation3d
import monologue.Annotations.Log
import monologue.Logged

// Angle values: - is outward, + is inward
private const val PIVOT_MOTOR_ID = 9
private const val PIVOT_ENCODER_ID = 0

private val PIVOT_SIM_STARTING_TRANSLATION = Translation3d(-0.32, 0.0, 0.72)
private val FORWARD_LIMIT: Angle = -1.636.radians
private val REVERSE_LIMIT: Angle = 1.8.radians
private val PID_TOLERANCE = 2.degrees
private val ABSOLUTE_ENCODER_OFFSET = -1.404.radians

// upright = 0 rad
object PivotAngle {
    val AMP = -0.72.radians
    val SOURCE = 0.radians
    val GROUND_INTAKE_HANDOFF = 0.9.radians
    val STOWED = GROUND_INTAKE_HANDOFF // same as of now
    val SPEAKER = 0.77.radians
    val STARTING = 1.15.radians // used to zero the encoder if there is no absolute encoder(we have one)
}

class Pivot(disable: Boolean = false): SubsystemBase(), Logged {
    private val motor: Motor
    private val absoluteEncoder: PositionEncoder?

    init {
        if (isSimulation() || disable){
            motor = MotorSim(DCMotor.getNEO(1), moi = 0.008.kilo.grams * (meters * meters))
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
            brakeWhenIdle = true,
            gearRatio = 96.0,
            positionPID = PIDConstants(1.0,0.0,0.001)
        )
        /*
        Trigger(DriverStation::isDisabled)
            .onTrue(InstantCommand {
                setIdle()
                motor.configure(brakeWhenIdle = false)
            }.ignoringDisable(true))
            .onFalse(InstantCommand {
                motor.configure(brakeWhenIdle = true)
            }.ignoringDisable(true))

         */
    }

    private val motionProfile: AngularMotionProfile? = null
    private var motionProfileSetpoint = AngularMotionProfileState(startingAngle)
    private val feedforward = AngularMotorFeedforward(0.0, 0.0, 0.0)

    @get:Log(key = "angle(Rad)")
    val angle get() = MathUtil.angleModulus(motor.encoder.angularPosition.inUnit(radians)).ofUnit(radians)

    @Log var atTarget = true
        private set

    // note: forward is negative, so we do <= forward limit
    @Log private fun willExceedSoftStop(movingForward: Boolean): Boolean =
        (angle <= FORWARD_LIMIT && movingForward) || (angle >= REVERSE_LIMIT && !movingForward)

    private fun resetMotionProfile(){
        if (motionProfile != null){
            motionProfileSetpoint.position = angle
        }
    }

    fun setIdle(){
        resetMotionProfile()
        motor.voltageOut = 0.volts
    }

    fun setVoltage(voltage: Voltage){
        resetMotionProfile()
        if (willExceedSoftStop(movingForward = voltage > 0.volts)){
            setIdle()
            atTarget = true
            return
        }
        motor.voltageOut = voltage
    }

    fun setSpeed(speed: Double) = setVoltage(speed * 12.volts)

    fun setAngle(target: Angle) {
        if (willExceedSoftStop(movingForward = this.angle > target)){
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
            log("Control/MotionProfileGoal(Rad)", target.inUnit(radians))
        }else{
            pidTarget = target
            ffVoltage = 0.volts
        }
        motor.setPositionSetpoint(pidTarget, ffVoltage)

        log("Control/Setpoint(Rad)", pidTarget.inUnit(radians))
        log("Control/FeedForward(Volts)", ffVoltage.inUnit(volts))
        atTarget = abs(target - this.angle) < PID_TOLERANCE
    }

    override fun periodic(){
        log("StatorCurrent", motor.statorCurrent.inUnit(amps))
        log("VoltageReading", motor.voltageOut.inUnit(volts))
        // logs Mechanism 3d pose for simulation.
        log(
            "Mechanism3dPose",
            Pose3d(
                PIVOT_SIM_STARTING_TRANSLATION,
                Rotation3d(0.degrees, -angle, 0.degrees) // custom overload function that accepts kmeasure quantities
            )
        )
    }
}