package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.hardware.sensors.encoders.Encoder

@Suppress("unused")
class ArmMotorSim(
    motorType: DCMotor,
    armLength: Distance,
    moi: MomentOfInertia = 0.000015.ofUnit(kilo.grams * (meters * meters)),
    motorToEncoderRatio: Double = 1.0,
    minimumEncoderMeasurement: Angle = Angle(Double.NEGATIVE_INFINITY),
    maximumEncoderMeasurement: Angle = Angle(Double.POSITIVE_INFINITY)
): MotorizedComponent {
    private val sim = SingleJointedArmSim(
        motorType,
        motorToEncoderRatio,
        moi.inUnit(kilo.grams * (meters * meters)),
        armLength.inUnit(meters),
        minimumEncoderMeasurement.inUnit(radians),
        maximumEncoderMeasurement.inUnit(radians),
        true, 0.0
    )

    private val positionController = SuperPIDController(
        PIDConstants(0,0,0),
        getInput = { encoder.angularPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts
    )

    private val continuousInputPositionController = SuperPIDController(
        PIDConstants(0,0,0),
        getInput = { encoder.angularPosition },
        continuousInputRange = 0.degrees..360.degrees,
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
            continuousInputPositionController.calculateOutput()
            velocityController.calculateOutput()
        }
    }

    override val encoder: Encoder = object: Encoder {
        override val angularPosition: Angle
            get() = sim.angleRads.ofUnit(radians)

        override val angularVelocity: AngularVelocity
            get() = sim.velocityRadPerSec.ofUnit(radians / seconds)
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
            continuousInputPositionController.constants = pidConstants
            continuousInputPositionController.target = rawPosition
            appliedVoltage = continuousInputPositionController.calculateOutput() + feedforward
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