package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.constants
import frc.chargers.hardware.motorcontrol.Motor
import kotlin.math.PI

/**
 * A base class for simulated motors.
 */
abstract class SimulatedMotorBase: Motor {
    abstract fun initializeWPILibSim(gearRatio: Double)

    private val positionController = PIDController(0.0, 0.0, 0.0)
    private val velocityController = PIDController(0.0, 0.0, 0.0)

    private var positionPIDConfigured = false
    private var velocityPIDConfigured = false

    override var inverted = false

    override fun setPositionSetpoint(position: Angle, feedforward: Voltage) {
        require(positionPIDConfigured){" You must specify a positionPID value using the configure() method. "}
        var encoderReading = this.encoder.angularPosition
        if (positionController.isContinuousInputEnabled) encoderReading %= 360.degrees

        val pidOutput = positionController.calculate(encoderReading.siValue, position.siValue)
        this.appliedVoltage = Voltage(pidOutput) + feedforward
    }

    override fun setVelocitySetpoint(velocity: AngularVelocity, feedforward: Voltage) {
        require(velocityPIDConfigured){" You must specify a velocityPID value using the configure() method. "}
        val pidOutput = velocityController.calculate(encoder.angularVelocity.siValue, velocity.siValue)
        this.appliedVoltage = Voltage(pidOutput) + feedforward
    }

    override fun configure(
        inverted: Boolean?,
        brakeWhenIdle: Boolean?,
        rampRate: Time?,
        statorCurrentLimit: Current?,
        followerMotors: List<Motor>,
        positionUpdateRate: Frequency?,
        velocityUpdateRate: Frequency?,
        optimizeUpdateRate: Boolean?,
        gearRatio: Double?,
        startingPosition: Angle?,
        positionPID: PIDConstants?,
        velocityPID: PIDConstants?,
        continuousInput: Boolean?
    ): SimulatedMotorBase {
        if (inverted != null) this.inverted = inverted
        require(followerMotors.isEmpty()) { "Don't add followers to a simulated motor; consider increasing the # of motors in the DCMotor.getNEO or DCMotor.getNeoVortex calls." }
        if (positionPID != null) positionController.constants = positionPID; positionPIDConfigured = true
        if (velocityPID != null) velocityController.constants = velocityPID; velocityPIDConfigured = true
        if (continuousInput == true) {
            positionController.enableContinuousInput(0.0, 2 * PI)
        } else if (continuousInput == false) {
            positionController.disableContinuousInput()
        }
        if (gearRatio != null) initializeWPILibSim(gearRatio)
        return this
    }
}