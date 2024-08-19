package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.controls.UnitPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.Encoder

class MotorSim(
    motorType: DCMotor,
    moi: MomentOfInertia = 0.00001.ofUnit(kilo.grams * (meters * meters)),
    motorToEncoderRatio: Double = 1.0,
): Motor {
    private val wpilibSim = DCMotorSim(
        motorType,
        motorToEncoderRatio,
        moi.inUnit(kilo.grams * (meters * meters))
    )

    private val positionController = UnitPIDController<AngleDimension, VoltageDimension>(
        0.0, 0.0, 0.0, outputRange = (-12).volts..12.volts
    )

    private val velocityController = UnitPIDController<AngularVelocityDimension, VoltageDimension>(
        0.0, 0.0, 0.0, outputRange = (-12).volts..12.volts
    )

    init{
        ChargerRobot.runPeriodic {
            wpilibSim.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        }
    }

    override val encoder: Encoder = SimEncoder()
    private inner class SimEncoder: Encoder {
        override val angularPosition: Angle
            get() = wpilibSim.angularPositionRad.ofUnit(radians)

        override val angularVelocity: AngularVelocity
            get() = wpilibSim.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override var hasInvert: Boolean = false

    override var appliedVoltage: Voltage = 0.volts
        set(value) {
            field = value
            wpilibSim.setInputVoltage(value.siValue * if (hasInvert) -1.0 else 1.0 )
        }

    override val statorCurrent: Current
        get() = wpilibSim.currentDrawAmps.ofUnit(amps)

    override fun setBrakeMode(shouldBrake: Boolean){}

    override fun setPositionSetpoint(
        rawPosition: Angle,
        pidConstants: PIDConstants,
        continuousInput: Boolean,
        feedforward: Voltage
    ) {
        if (continuousInput){
            positionController.enableContinuousInput(0.degrees..360.degrees)
        }else{
            positionController.disableContinuousInput()
        }
        positionController.constants = pidConstants
        appliedVoltage = positionController.calculate(encoder.angularPosition, rawPosition, feedforward)
    }

    override fun setVelocitySetpoint(rawVelocity: AngularVelocity, pidConstants: PIDConstants, feedforward: Voltage) {
        velocityController.constants = pidConstants
        appliedVoltage = velocityController.calculate(encoder.angularVelocity, rawVelocity, feedforward)
    }
}