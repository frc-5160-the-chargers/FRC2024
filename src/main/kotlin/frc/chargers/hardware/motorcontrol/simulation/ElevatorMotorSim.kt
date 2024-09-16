package frc.chargers.hardware.motorcontrol.simulation

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Represents 1 or a group of motors powering an elevator.
 * Acts as wrapper around WPILib's [ElevatorSim] that extends the [frc.chargers.hardware.motorcontrol.Motor] interface.
 */
class ElevatorMotorSim(
    private val motorType: DCMotor,
    private val carriageMass: Mass,
    private val lowestPosition: Angle = Angle(Double.NEGATIVE_INFINITY),
    private val highestPosition: Angle = Angle(Double.POSITIVE_INFINITY)
): SimulatedMotorBase() {
    lateinit var base: ElevatorSim
        private set

    init {
        initializeWPILibSim(1.0) // initializes the sim
        if (RobotBase.isSimulation()) {
            ChargerRobot.runPeriodic {
                base.update(ChargerRobot.LOOP_PERIOD.inUnit(seconds))
            }
        }
    }

    override fun initializeWPILibSim(gearRatio: Double) {
        base = ElevatorSim(
            motorType,
            gearRatio,
            carriageMass.inUnit(kilo.grams),
            /*"Drum Radius"*/ 1.0 / (2.0 * Math.PI), // This value ensures that the circumference = 1 meters
            /*Min Height*/ (lowestPosition * 1.meters).inUnit(meters),
            /*Max Height*/ (highestPosition * 1.meters).inUnit(meters),
            /*Simulate Gravity*/ true,
            0.0
        )
    }

    override val encoder: Encoder = SimEncoder()
    private inner class SimEncoder : Encoder {
        override val angularPosition: Angle
            get() = base.positionMeters.ofUnit(meters) / 1.meters

        override val angularVelocity: AngularVelocity
            get() = base.velocityMetersPerSecond.ofUnit(meters / seconds) / 1.meters
    }

    override var appliedVoltage: Voltage = 0.volts
        set(value) {
            field = value
            base.setInputVoltage(value.siValue * if (inverted) -1.0 else 1.0)
        }

    override val statorCurrent: Current
        get() = base.currentDrawAmps.ofUnit(amps)
}