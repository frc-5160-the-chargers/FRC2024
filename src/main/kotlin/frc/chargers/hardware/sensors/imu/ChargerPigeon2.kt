@file:Suppress("unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.hertz
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.framework.logged


/**
 * A wrapper around the [Pigeon2] class from CTRE,
 * with units support and interface implementation.
 */
class ChargerPigeon2(
    val deviceID: Int,
    canBus: String? = null,
    factoryDefault: Boolean = true,
    headingUpdateFrequency: Frequency? = null,
    var simHeadingSource: () -> Angle = { Angle(0.0) }
): ZeroableHeadingProvider {
    val base: Pigeon2 = if (canBus == null) Pigeon2(deviceID) else Pigeon2(deviceID, canBus)

    private val yawSignal = base.yaw
    private val rollSignal = base.roll
    private val pitchSignal = base.pitch

    private val yawRateSignal = base.angularVelocityXWorld
    private val pitchRateSignal = base.angularVelocityYWorld
    private val rollRateSignal = base.angularVelocityXWorld

    private val xAccelSignal = base.accelerationX
    private val yAccelSignal = base.accelerationY
    private val zAccelSignal = base.accelerationZ

    private val allSignals = listOf(
        yawSignal, rollSignal, pitchSignal,
        yawRateSignal, pitchRateSignal, rollRateSignal,
        xAccelSignal, yAccelSignal, zAccelSignal
    )
    private var simPreviousYaw = Angle(0.0)

    init {
        if (factoryDefault) base.configurator.apply(Pigeon2Configuration())
        if (headingUpdateFrequency != null) yawSignal.setUpdateFrequency(headingUpdateFrequency.inUnit(hertz))
        zeroHeading()
    }

    /**
     * The heading of the Pigeon; equivalent to yaw.
     */
    override val heading: Angle get() = this.yaw

    /**
     * Zeroes the heading of the Pigeon.
     */
    override fun zeroHeading(angle: Angle) {
        base.setYaw(angle.inUnit(degrees))
    }

    val yaw: Angle by logged {
        if (isReal()) yawSignal.refresh().value.ofUnit(degrees) else simHeadingSource()
    }

    val pitch: Angle by logged {
        if (isReal()) pitchSignal.refresh().value.ofUnit(degrees) else Angle(0.0)
    }

    val roll: Angle by logged {
        if (isReal()) rollSignal.refresh().value.ofUnit(degrees) else Angle(0.0)
    }

    val yawRate: AngularVelocity by logged {
        if (isReal()) {
            yawRateSignal.refresh().value.ofUnit(degrees / seconds)
        } else {
            val currH = simHeadingSource()
            ((currH - simPreviousYaw) / 0.02.seconds).also {
                simPreviousYaw = currH
            }
        }
    }

    val pitchRate: AngularVelocity by logged {
        if (isReal()) pitchRateSignal.refresh().value.ofUnit(degrees / seconds) else AngularVelocity(0.0)
    }

    val rollRate: AngularVelocity by logged {
        if (isReal()) rollRateSignal.refresh().value.ofUnit(degrees / seconds) else AngularVelocity(0.0)
    }

    val xAcceleration: Acceleration by logged {
        if (isReal()) xAccelSignal.refresh().value.ofUnit(standardGravities) else Acceleration(0.0)
    }

    val yAcceleration: Acceleration by logged {
        if (isReal()) yAccelSignal.refresh().value.ofUnit(standardGravities) else Acceleration(0.0)
    }

    val zAcceleration: Acceleration by logged {
        if (isReal()) zAccelSignal.refresh().value.ofUnit(standardGravities) else Acceleration(0.0)
    }

    /**
     * Determines if the gyro is connected or not.
     */
    val isConnected: Boolean by logged {
        allSignals.all { it.status == StatusCode.OK }
    }
}