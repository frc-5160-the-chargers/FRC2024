@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.configuration.safeConfigure
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.wpilibextensions.delay
import com.ctre.phoenix6.configs.Pigeon2Configuration

/**
 * Creates a [ChargerPigeon2] with inline configuration.
 */
public inline fun ChargerPigeon2(
    canId: Int,
    canBus: String = "rio",
    factoryDefault: Boolean = true,
    configure: ChargerPigeon2Configuration.() -> Unit
): ChargerPigeon2 = ChargerPigeon2(
    canId, canBus, factoryDefault,
    ChargerPigeon2Configuration().apply(configure)
)


/**
 * A wrapper around the [Pigeon2] class from CTRE,
 * with units support and interface implementation.
 *
 * @see ChargerPigeon2Configuration
 */
public class ChargerPigeon2(
    canId: Int,
    canBus: String = "rio",
    factoryDefault: Boolean = true,
    configuration: ChargerPigeon2Configuration? = null
): Pigeon2(canId, canBus), ZeroableHeadingProvider, HardwareConfigurable<ChargerPigeon2Configuration> {

    init{
        ChargerRobot.runPeriodically (addToFront = true) {
            val currentStatus = BaseStatusSignal.refreshAll(
                *gyroscope.getSignals(),
                *accelerometer.getSignals()
            )
            isConnected = currentStatus == StatusCode.OK
        }

        val baseConfig = Pigeon2Configuration()

        if (!factoryDefault){
            configurator.refresh(baseConfig)
        }else{
            println("Pigeon2 will factory default.")
        }

        if (configuration != null){
            configure(configuration, baseConfig)
        }else{
            configure(ChargerPigeon2Configuration(), baseConfig)
        }
    }



    /**
     * The gyroscope of the Pigeon; contains yaw, pitch, and roll data
     * as well as the recorded rate(s) for those values.
     */
    public val gyroscope: Gyroscope = Gyroscope()

    /**
     * The accelerometer of the Pigeon; contains x, y, and z acceleration.
     */
    public val accelerometer: Accelerometer = Accelerometer()

    /**
     * The heading of the Pigeon; equivalent to yaw.
     */
    override val heading: Angle by ImuLog.quantity{ gyroscope.yaw }

    /**
     * Zeroes the heading of the Pigeon.
     */
    override fun zeroHeading() { reset() }

    private var _isConnected = true

    /**
     * Determines if the gyro is connected or not.
     */
    public var isConnected: Boolean by ImuLog.boolean(
        getValue = { isReal() &&  _isConnected  },
        setValue = { value -> _isConnected = value }
    )


    /*
    Internal constructor makes it so that the inner class can be accepted as a type argument,
    but can't be instantiated.
     */


    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope {

        private val yawSignal = getYaw()
        private val rollSignal = getRoll()
        private val pitchSignal = getPitch()

        private val yawRateSignal = angularVelocityXWorld
        private val pitchRateSignal = angularVelocityYWorld
        private val rollRateSignal = angularVelocityXWorld

        internal fun getSignals(): Array<BaseStatusSignal> =
            arrayOf(yawSignal, rollSignal, pitchSignal, yawRateSignal, pitchRateSignal, rollRateSignal)

        private var simPreviousYaw = Angle(0.0)

        override val yaw: Angle by GyroLog.quantity{
            if (isReal()) yawSignal.value.ofUnit(degrees) else getSimHeading()
        }

        override val pitch: Angle by GyroLog.quantity{
            if (isReal()) pitchSignal.value.ofUnit(degrees) else Angle(0.0)
        }

        override val roll: Angle by GyroLog.quantity{
            if (isReal()) rollSignal.value.ofUnit(degrees) else Angle(0.0)
        }

        public val yawRate: AngularVelocity by GyroLog.quantity{
            if (isReal()){
                yawRateSignal.value.ofUnit(degrees/seconds)
            }else{
                val currH = getSimHeading()
                ((currH - simPreviousYaw) / ChargerRobot.LOOP_PERIOD).also{
                    simPreviousYaw = currH
                }
            }
        }

        public val pitchRate: AngularVelocity by GyroLog.quantity{
            if (isReal()) pitchRateSignal.value.ofUnit(degrees/seconds) else AngularVelocity(0.0)
        }

        public val rollRate: AngularVelocity by GyroLog.quantity{
            if (isReal()) rollRateSignal.value.ofUnit(degrees/seconds) else AngularVelocity(0.0)
        }

    }



    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer {
        private val xAccelSignal = accelerationX
        private val yAccelSignal = accelerationY
        private val zAccelSignal = accelerationZ

        internal fun getSignals(): Array<BaseStatusSignal> =
            arrayOf(xAccelSignal, yAccelSignal, zAccelSignal)

        override val xAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) xAccelSignal.value.ofUnit(standardGravities) else Acceleration(0.0)
        }

        override val yAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) yAccelSignal.value.ofUnit(standardGravities) else Acceleration(0.0)
        }

        override val zAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) zAccelSignal.value.ofUnit(standardGravities) else Acceleration(0.0)
        }
    }

    private val allConfigErrors: LinkedHashSet<StatusCode> = linkedSetOf()
    private var configAppliedProperly = true

    private fun StatusCode.updateConfigStatus(): StatusCode {
        if (this != StatusCode.OK){
            if (isSimulation()){
                println("A Phoenix Device did not configure properly; however, this was ignored because the code is running in simulation.")
            }else{
                delay(200.milli.seconds)
                allConfigErrors.add(this)
                configAppliedProperly = false
            }
        }
        return this
    }


    override fun configure(configuration: ChargerPigeon2Configuration) {
        val baseConfig = Pigeon2Configuration()
        configurator.refresh(baseConfig)
        configure(configuration, baseConfig)
    }


    public fun configure(configuration: ChargerPigeon2Configuration, basePigeon2Configuration: Pigeon2Configuration){
        configAppliedProperly = true
        safeConfigure(
            deviceName = "ChargerCANcoder(id = $deviceID)",
            getErrorInfo = {"All Recorded Errors: $allConfigErrors"}
        ) {
            allConfigErrors.clear()
            applyChanges(basePigeon2Configuration, configuration)
            configurator.apply(basePigeon2Configuration).updateConfigStatus()
            return@safeConfigure configAppliedProperly
        }
    }

}

/**
 * A configuration class for the [ChargerPigeon2].
 */
public data class ChargerPigeon2Configuration(
    var futureProofConfigs: Boolean? = null,
    var gyroScalarX: Angle? = null,
    var gyroScalarY: Angle? = null,
    var gyroScalarZ: Angle? = null,
    var mountPosePitch: Angle? = null,
    var mountPoseYaw: Angle? = null,
    var mountPoseRoll: Angle? = null,
    var disableNoMotionCalibration: Boolean? = null,
    var disableTemperatureCompensation: Boolean? = null,
    var enableCompass: Boolean? = null
): HardwareConfiguration

internal fun applyChanges(ctreConfig: Pigeon2Configuration, configuration: ChargerPigeon2Configuration): Pigeon2Configuration{
    ctreConfig.apply{
        configuration.futureProofConfigs?.let{ FutureProofConfigs = it }
        GyroTrim.apply{
            configuration.gyroScalarX?.let{ GyroScalarX = it.inUnit(degrees) }
            configuration.gyroScalarY?.let{ GyroScalarY = it.inUnit(degrees) }
            configuration.gyroScalarZ?.let{ GyroScalarZ = it.inUnit(degrees) }
        }

        MountPose.apply{
            configuration.mountPosePitch?.let{ MountPosePitch = it.inUnit(degrees) }
            configuration.mountPoseYaw?.let{ MountPoseYaw = it.inUnit(degrees) }
            configuration.mountPoseRoll?.let{ MountPoseRoll = it.inUnit(degrees) }
        }

        Pigeon2Features.apply{
            configuration.disableTemperatureCompensation?.let{DisableTemperatureCompensation = it}
            configuration.disableNoMotionCalibration?.let{DisableNoMotionCalibration = it}
            configuration.enableCompass?.let{EnableCompass = it}
        }
    }
    return ctreConfig
}