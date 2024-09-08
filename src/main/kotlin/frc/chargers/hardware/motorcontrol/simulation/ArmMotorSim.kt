package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Simulates 1(or a group of) motors on a single arm joint.
 * Acts as wrapper around WPILib's [SingleJointedArmSim] that extends the [frc.chargers.hardware.motorcontrol.Motor] interface.
 */
@Suppress("unused")
class ArmMotorSim(
    private val motorType: DCMotor,
    private val armLength: Distance,
    private val moi: MomentOfInertia,
    private val angleRange: ClosedRange<Angle> = Angle(Double.NEGATIVE_INFINITY)..Angle(Double.POSITIVE_INFINITY)
): SimulatedMotorBase() {
    lateinit var base: SingleJointedArmSim
        private set

    init {
        initializeWPILibSim(1.0)
        ChargerRobot.runPeriodic {
            base.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
        }
    }

    override fun initializeWPILibSim(gearRatio: Double) {
        base = SingleJointedArmSim(
            motorType,
            gearRatio,
            moi.inUnit(kilo.grams * (meters * meters)),
            armLength.inUnit(meters),
            angleRange.start.inUnit(radians),
            angleRange.endInclusive.inUnit(radians),
            true, 0.0
        )
    }

    override val encoder: Encoder = SimEncoder()
    private inner class SimEncoder: Encoder {
        override val angularPosition: Angle
            get() = base.angleRads.ofUnit(radians)

        override val angularVelocity: AngularVelocity
            get() = base.velocityRadPerSec.ofUnit(radians / seconds)
    }

    override var appliedVoltage: Voltage = 0.volts
        set(value) {
            field = value
            base.setInputVoltage(value.siValue * if (super.inverted) -1.0 else 1.0)
            super.followers.forEach{ it.appliedVoltage = value }
        }

    override val statorCurrent: Current
        get() = base.currentDrawAmps.ofUnit(amps)
}