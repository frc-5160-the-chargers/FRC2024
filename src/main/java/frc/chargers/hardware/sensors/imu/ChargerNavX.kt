@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.imu.gyroscopes.ThreeAxisGyroscope
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.kinematics.xVelocity
import frc.chargers.wpilibextensions.kinematics.yVelocity


public class ChargerNavX(
    private val useFusedHeading: Boolean = true,
    public val ahrs: AHRS = AHRS(),
): ZeroableHeadingProvider {

    /**
     * The specified LogTabs below are in fact LoggableInputsProvider's,
     * which automatically handle AdvantageKit logging and replay support(and work essentially as getters).
     * @see [frc.chargers.advantagekitextensions.LoggableInputsProvider]
     */

    /**
     * Creates a new NavX.
     */
    init {
        zeroHeading()
    }

    private var headingOffset = 0.0.degrees

    override fun zeroHeading(){
        ahrs.reset()
        headingOffset = if (isReal()) {
            -ahrs.fusedHeading.toDouble().ofUnit(degrees)
        }else{
            getSimHeading().inputModulus(0.degrees..360.degrees)
        }
        println("NavX YAW has been zeroed.")
    }

    public val firmwareVersion: String get() = ahrs.firmwareVersion

    public val isConnected: Boolean by ImuLog.boolean{ ahrs.isConnected && isReal() }

    override val heading: Angle by ImuLog.quantity{
        if (isReal()) {
            if (ahrs.isMagnetometerCalibrated && !ahrs.isMagneticDisturbance && useFusedHeading){
                -ahrs.fusedHeading.toDouble().ofUnit(degrees) - headingOffset
                // Negative sign because the navX reports clockwise as positive, whereas we want counterclockwise to be positive
            }else{
                ahrs.angle.ofUnit(degrees)
            }
        }else{
            (getSimHeading() - headingOffset).inputModulus(0.degrees..360.degrees)
        }
    }

    public val altitude: Distance? by ImuLog.nullableQuantity{
        if (ahrs.isAltitudeValid && isReal()) ahrs.altitude.toDouble().ofUnit(meters) else null
    }

    public val gyroscope: Gyroscope = Gyroscope()
    public val accelerometer: Accelerometer = Accelerometer()
    public val speedometer: Speedometer = Speedometer()

    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope {
        override val yaw: Angle by GyroLog.quantity{
            if (isReal()) ahrs.yaw.toDouble().ofUnit(degrees) else getSimHeading().inputModulus(-180.degrees..180.degrees)
        }

        override val pitch: Angle by GyroLog.quantity{
            if (isReal()) ahrs.pitch.toDouble().ofUnit(degrees) else Angle(0.0)
        }

        override val roll: Angle by GyroLog.quantity{
            if (isReal()) ahrs.roll.toDouble().ofUnit(degrees) else Angle(0.0)
        }

        override val heading: Angle by GyroLog.quantity{ this@ChargerNavX.heading }
    }

    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer {

        private var previousXVelSim = Velocity(0.0)
        private var previousYVelSim = Velocity(0.0)


        override val xAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()){
                return@quantity ahrs.worldLinearAccelX.toDouble().ofUnit(standardGravities)
            }else{
                val currXVelSim = getSimChassisSpeeds().xVelocity
                return@quantity ((currXVelSim - previousXVelSim) / ChargerRobot.LOOP_PERIOD).also{
                    previousXVelSim = currXVelSim
                }
            }
        }

        override val yAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()){
                return@quantity ahrs.worldLinearAccelY.toDouble().ofUnit(standardGravities)
            }else{
                val currYVelSim = getSimChassisSpeeds().yVelocity
                return@quantity ((currYVelSim - previousYVelSim) / ChargerRobot.LOOP_PERIOD).also{
                    previousYVelSim = currYVelSim
                }
            }
        }

        override val zAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()) ahrs.worldLinearAccelZ.toDouble().ofUnit(standardGravities) else Acceleration(0.0)
        }
    }

    public inner class Speedometer internal constructor(): ThreeAxisSpeedometer {
        // uses custom getters(ChassisSpeeds.xVelocity, ChassisSpeeds.yVelocity)
        // which return kmeasure units for sim
        override val xVelocity: Velocity by SpeedometerLog.quantity{
            if (isReal()) ahrs.velocityX.toDouble().ofUnit(meters / seconds) else getSimChassisSpeeds().xVelocity
        }

        override val yVelocity: Velocity by SpeedometerLog.quantity{
            if (isReal()) ahrs.velocityY.toDouble().ofUnit(meters / seconds) else getSimChassisSpeeds().yVelocity
        }

        override val zVelocity: Velocity by SpeedometerLog.quantity{
            if (isReal()) ahrs.velocityZ.toDouble().ofUnit(meters / seconds) else Velocity(0.0)
        }
    }
}