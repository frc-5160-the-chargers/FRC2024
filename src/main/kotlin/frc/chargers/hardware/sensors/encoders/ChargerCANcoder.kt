package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.SensorDirectionValue

class ChargerCANcoder(
    val deviceID: Int,
    factoryDefault: Boolean = true,
    sensorDirection: SensorDirectionValue? = null,
    absoluteSensorRange: AbsoluteSensorRangeValue? = null,
    magnetOffset: Angle? = null,
): Encoder {
    /**
     * The base [CANcoder] class.
     */
    val base: CANcoder = CANcoder(deviceID)

    init {
        val config = CANcoderConfiguration()
        if (!factoryDefault) base.configurator.refresh(config)
        if (sensorDirection != null) config.MagnetSensor.SensorDirection = sensorDirection
        if (absoluteSensorRange != null) config.MagnetSensor.AbsoluteSensorRange = absoluteSensorRange
        if (magnetOffset != null) config.MagnetSensor.MagnetOffset = magnetOffset.inUnit(rotations)
        base.configurator.apply(config)
    }

    private val posSignal = base.position
    private var velSignal = base.velocity
    private val absolutePosSignal = base.absolutePosition

    /**
     * Represents the absolute encoder of the CANcoder.
     */
    val absolute: Encoder = AbsoluteEncoderAdaptor()
    private inner class AbsoluteEncoderAdaptor: Encoder by this {
        override val angularPosition: Angle
            get() = absolutePosSignal.refresh(true).value.ofUnit(rotations)
    }

    /**
     * Obtains the relative position from the CANcoder.
     */
    override val angularPosition: Angle
        get() = posSignal.refresh(true).value.ofUnit(rotations)

    /**
     * Obtains the velocity of the CANcoder.
     */
    override val angularVelocity: AngularVelocity
        get() = velSignal.refresh(true).value.ofUnit(rotations/ seconds)
}