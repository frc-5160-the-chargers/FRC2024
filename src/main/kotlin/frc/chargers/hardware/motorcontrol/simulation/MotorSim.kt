package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Simulates one or a group of motors that are not affected by gravity.
 * Acts as wrapper around WPILib's [DCMotorSim] that extends the [frc.chargers.hardware.motorcontrol.Motor] interface.
 */
@Suppress("unused")
class MotorSim(
    private val motorType: DCMotor,
    private val moi: MomentOfInertia = 0.004.kilo.grams * (meters * meters), // good estimate if you don't care about accuracy
): SimulatedMotorBase() {
    lateinit var base: DCMotorSim
        private set

    init {
        initializeWPILibSim(1.0) // initializes the sim
        if (RobotBase.isSimulation()) {
            ChargerRobot.runPeriodic { base.update(0.02) }
        }
    }

    override fun initializeWPILibSim(gearRatio: Double) {
        base = DCMotorSim(motorType, gearRatio, moi.inUnit(kilo.grams * (meters * meters)))
    }

    override val encoder: Encoder = SimEncoder()
    private inner class SimEncoder : Encoder {
        override val angularPosition: Angle
            get() = base.angularPositionRad.ofUnit(radians)

        override val angularVelocity: AngularVelocity
            get() = base.angularVelocityRadPerSec.ofUnit(radians / seconds)
    }

    override var appliedVoltage: Voltage = 0.volts
        set(value) {
            field = value
            base.setInputVoltage(value.siValue * if (super.inverted) -1.0 else 1.0)
        }

    override val statorCurrent: Current
        get() = base.currentDrawAmps.ofUnit(amps)
}
