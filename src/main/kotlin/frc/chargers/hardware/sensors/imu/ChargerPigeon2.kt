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
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.Loggable
import limelight.LimelightHelpers


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
): ZeroableHeadingProvider, Loggable {
    override val namespace = "Pigeon2"

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

    init {
        if (factoryDefault) base.configurator.apply(Pigeon2Configuration())
        if (headingUpdateFrequency != null) yawSignal.setUpdateFrequency(headingUpdateFrequency.inUnit(hertz))
    }

    /**
     * The gyroscope of the Pigeon; contains yaw, pitch, and roll data
     * as well as the recorded rate(s) for those values.
     */
    val gyroscope: Gyroscope = Gyroscope()
    inner class Gyroscope internal constructor() : ThreeAxisGyroscope, Loggable {
        /*
        Internal constructor makes it so that the inner class can be accepted as a type argument,
        but can't be instantiated.
         */
        override val namespace = "Pigeon2/Gyroscope"
        private var simPreviousYaw = Angle(0.0)

        override val yaw: Angle by logged {
            if (isReal()) yawSignal.refresh().value.ofUnit(degrees) else simHeadingSource()
        }

        override val pitch: Angle by logged {
            if (isReal()) pitchSignal.refresh().value.ofUnit(degrees) else Angle(0.0)
        }

        override val roll: Angle by logged {
            if (isReal()) rollSignal.refresh().value.ofUnit(degrees) else Angle(0.0)
        }

        val yawRate: AngularVelocity by logged {
            if (isReal()) {
                yawRateSignal.refresh().value.ofUnit(degrees / seconds)
            } else {
                val currH = simHeadingSource()
                ((currH - simPreviousYaw) / ChargerRobot.LOOP_PERIOD).also {
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
    }

    /**
     * The accelerometer of the Pigeon; contains x, y, and z acceleration.
     */
    val accelerometer: Accelerometer = Accelerometer()
    inner class Accelerometer internal constructor() : ThreeAxisAccelerometer, Loggable {
        override val namespace = "Pigeon2/Accelerometer"

        override val xAcceleration: Acceleration by logged {
            if (isReal()) xAccelSignal.refresh().value.ofUnit(standardGravities) else Acceleration(0.0)
        }

        override val yAcceleration: Acceleration by logged {
            if (isReal()) yAccelSignal.refresh().value.ofUnit(standardGravities) else Acceleration(0.0)
        }

        override val zAcceleration: Acceleration by logged {
            if (isReal()) zAccelSignal.refresh().value.ofUnit(standardGravities) else Acceleration(0.0)
        }
    }

    /**
     * The heading of the Pigeon; equivalent to yaw.
     */
    override val heading: Angle by logged { gyroscope.yaw }

    /**
     * Zeroes the heading of the Pigeon.
     */
    override fun zeroHeading(angle: Angle) {
        base.setYaw(angle.inUnit(degrees))
    }

    /**
     * Determines if the gyro is connected or not.
     */
    val isConnected: Boolean by logged {
        allSignals.all { it.status == StatusCode.OK }
    }

    /**
     * Broadcasts robot orientation for the MegaTag2 system.
     * Should be run periodically; either in a periodic() method or using [ChargerRobot.runPeriodic].
     */
    fun broadcastOrientationForMegaTag2(
        vararg limelightNames: String
    ) {
        if (isSimulation()) return
        for (llName in limelightNames) {
            LimelightHelpers.setRobotOrientation(
                llName,
                heading.inUnit(degrees),
                gyroscope.yawRate.inUnit(degrees / seconds),
                gyroscope.pitch.inUnit(degrees),
                gyroscope.pitchRate.inUnit(degrees / seconds),
                gyroscope.roll.inUnit(degrees),
                gyroscope.rollRate.inUnit(degrees / seconds),
            )
        }
    }
}