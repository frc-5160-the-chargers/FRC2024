package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.hardware.sensors.encoders.Encoder

@Suppress("unused")
class ElevatorMotorSim(
    motorType: DCMotor,
    carriageMass: Mass,
    motorToEncoderRatio: Double = 1.0,
    minimumEncoderMeasurement: Angle = Angle(Double.NEGATIVE_INFINITY),
    maximumEncoderMeasurement: Angle = Angle(Double.POSITIVE_INFINITY)
): MotorizedComponent {
    private val sim = ElevatorSim(
        motorType,
        motorToEncoderRatio,
        carriageMass.inUnit(kilo.grams),
        /*"Drum Radius"*/ 1.0 / (2.0 * Math.PI), // This value ensures that the circumference = 1 meters
        /*Min Height*/ (minimumEncoderMeasurement * 1.meters).inUnit(meters),
        /*Max Height*/ (maximumEncoderMeasurement * 1.meters).inUnit(meters),
        /*Simulate Gravity*/ true,
        0.0
    )

    private val positionController = SuperPIDController(
        PIDConstants(0,0,0),
        getInput = { encoder.angularPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )

    private val velocityController = SuperPIDController(
        PIDConstants(0,0,0),
        getInput = { encoder.angularVelocity },
        target = 0.degrees / 0.seconds,
        outputRange = (-12).volts..12.volts
    )

    init{
        ChargerRobot.runPeriodically {
            sim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            positionController.calculateOutput()
            velocityController.calculateOutput()
        }
    }

    override val encoder: Encoder = object: Encoder {
        // drum radius is specified to be 1 meters; thus, we divide by that to get angular position/velocity
        override val angularPosition: Angle
            get() = sim.positionMeters.ofUnit(meters) / 1.meters

        override val angularVelocity: AngularVelocity
            get() = sim.velocityMetersPerSecond.ofUnit(meters / seconds) / 1.meters
    }

    override var hasInvert: Boolean = false

    override var appliedVoltage: Voltage = 0.volts
        set(value) {
            field = value
            sim.setInputVoltage(value.siValue * if (hasInvert) -1.0 else 1.0 )
        }

    override val statorCurrent: Current
        get() = sim.currentDrawAmps.ofUnit(amps)

    override fun setPositionSetpoint(
        rawPosition: Angle,
        pidConstants: PIDConstants,
        continuousInput: Boolean,
        feedforward: Voltage
    ) {
        if (continuousInput){
            error("Elevators do not support continuous input.")
        }else{
            positionController.constants = pidConstants
            positionController.target = rawPosition
            appliedVoltage = positionController.calculateOutput() + feedforward
        }
    }

    override fun setVelocitySetpoint(rawVelocity: AngularVelocity, pidConstants: PIDConstants, feedforward: Voltage) {
        velocityController.constants = pidConstants
        velocityController.target = rawVelocity
        appliedVoltage = positionController.calculateOutput() + feedforward
    }
}