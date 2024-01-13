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
    private val useFusedHeading: Boolean = false,
    public val ahrs: AHRS = AHRS(),
): ZeroableHeadingProvider {
    /**
     * ImuLog, GyroscopeLog, AccelerometerLog, and SpeedometerLog are LoggableInputsProviders,
     * which automatically handle AdvantageKit logging and replay support(and work essentially as getters variables).
     * @see [frc.chargers.advantagekitextensions.LoggableInputsProvider]
     */

    private var fusedHeadingOffset = 0.degrees
    private var simBasicHeadingOffset = 0.degrees



    /**
     * Zeroes the heading of the NavX.
     *
     * Does not zero the yaw; call navX.gyroscope.zeroYaw() if this is desired instead.
     */
    override fun zeroHeading(angle: Angle){
        if (isReal()){
            if (ahrs.isCalibrating){
                println("Waiting for AHRS to calibrate...")
                // Wait for 1 second (hardware initialization) before zeroing heading
                Thread.sleep(1000)
                println("AHRS has finished calibrating.")
            }

            ahrs.angleAdjustment = -angle.inUnit(degrees)
            println("AHRS angle/heading has been zeroed. This does not impact the gyroscope yaw(AHRS.getYaw / navX.gyroscope.yaw).")

            // negated because fused heading direction is inverted; see below
            fusedHeadingOffset = ahrs.fusedHeading.toDouble().ofUnit(degrees)
            println("AHRS fused heading has been zeroed. This does not impact the gyroscope yaw(AHRS.getYaw / navX.gyroscope.yaw).")
        }else{
            fusedHeadingOffset = -IMUSimulation.getHeading()
            simBasicHeadingOffset = -IMUSimulation.getHeading()
        }
    }

    public val firmwareVersion: String by ImuLog.string{ ahrs.firmwareVersion }

    /**
     * Returns if the NavX is connected or not. Automatically false during simulation.
     */
    public val isConnected: Boolean by ImuLog.boolean{ ahrs.isConnected && isReal() }

    /**
     * The heading of the NavX. Reports a value in between 0-360 degrees.
     */
    override val heading: Angle by ImuLog.quantity{
        if (isReal()) {
            if (ahrs.isMagnetometerCalibrated && !ahrs.isMagneticDisturbance && useFusedHeading){
                -ahrs.fusedHeading.toDouble().ofUnit(degrees) + fusedHeadingOffset
                // Negative sign because the navX reports clockwise as positive, whereas we want counterclockwise to be positive
            }else{
                ahrs.angle.ofUnit(degrees)
            }
        }else{
            (IMUSimulation.getHeading() + simBasicHeadingOffset)
        }.inputModulus(0.degrees..360.degrees)
    }

    /**
     * The altitude from the NavX. A null value represents an invalid altitude.
     */
    public val altitude: Distance? by ImuLog.nullableQuantity{
        if (ahrs.isAltitudeValid && isReal()) ahrs.altitude.toDouble().ofUnit(meters) else null
    }

    /**
     * The gyroscope of the NavX. Implements [ThreeAxisGyroscope] and HeadingProvider.
     */
    public val gyroscope: Gyroscope = Gyroscope()

    /**
     * The accelerometer of the NavX. Implements [ThreeAxisAccelerometer].
     */
    public val accelerometer: Accelerometer = Accelerometer()

    /**
     * The speedometer of the NavX. Implements [ThreeAxisSpeedometer].
     */
    public val speedometer: Speedometer = Speedometer()





    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope {
        override val yaw: Angle by GyroLog.quantity{
            if (isReal()) {
                ahrs.yaw.toDouble().ofUnit(degrees)
            } else {
                IMUSimulation.getHeading().inputModulus(-180.degrees..180.degrees)
            }
        }

        override val pitch: Angle by GyroLog.quantity{
            if (isReal()) ahrs.pitch.toDouble().ofUnit(degrees) else Angle(0.0)
        }

        override val roll: Angle by GyroLog.quantity{
            if (isReal()) ahrs.roll.toDouble().ofUnit(degrees) else Angle(0.0)
        }

        public fun zeroYaw(){
            ahrs.zeroYaw()
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
                val currXVelSim = IMUSimulation.getChassisSpeeds().xVelocity
                return@quantity ((currXVelSim - previousXVelSim) / ChargerRobot.LOOP_PERIOD).also{
                    previousXVelSim = currXVelSim
                }
            }
        }

        override val yAcceleration: Acceleration by AccelerometerLog.quantity{
            if (isReal()){
                return@quantity ahrs.worldLinearAccelY.toDouble().ofUnit(standardGravities)
            }else{
                val currYVelSim = IMUSimulation.getChassisSpeeds().yVelocity
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
            if (isReal()) ahrs.velocityX.toDouble().ofUnit(meters / seconds) else IMUSimulation.getChassisSpeeds().xVelocity
        }

        override val yVelocity: Velocity by SpeedometerLog.quantity{
            if (isReal()) ahrs.velocityY.toDouble().ofUnit(meters / seconds) else IMUSimulation.getChassisSpeeds().yVelocity
        }

        override val zVelocity: Velocity by SpeedometerLog.quantity{
            if (isReal()) ahrs.velocityZ.toDouble().ofUnit(meters / seconds) else Velocity(0.0)
        }
    }
}