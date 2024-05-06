@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.Loggable
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.kinematics.xVelocity
import frc.chargers.wpilibextensions.kinematics.yVelocity
import frc.external.limelight.LimelightHelpers


@Suppress("LeakingThis")
public open class ChargerNavX(
    private val useFusedHeading: Boolean = false,
    public val ahrs: AHRS = AHRS(),
): ZeroableHeadingProvider, Loggable {
    override val namespace = "NavX"
    private var headingOffset by logged(0.degrees)

    /**
     * Zeroes the heading of the NavX.
     *
     * Does not zero the yaw; call navX.gyroscope.zeroYaw() if this is desired instead.
     */
    override fun zeroHeading(angle: Angle){
        if (isReal()){
            while (ahrs.isCalibrating){
                println("Waiting for AHRS to calibrate...")
                // Wait for 1 second (hardware initialization) before zeroing heading
                Thread.sleep(1000)
            }

            println("AHRS has finished calibrating.")

            headingOffset = -heading + headingOffset + angle

            println("AHRS angle and fused heading has been offset. This does not impact ahrs.getYaw()!")
        }else{
            headingOffset = -IMUSimulation.getHeading()
            println("AHRS heading has been offset in sim.")
        }
    }

    /**
     * Broadcasts robot orientation for the MegaTag2 system.
     * Should be run periodically; either in a periodic() method or using [ChargerRobot.runPeriodic].
     */
    fun broadcastOrientationForMegaTag2(
        vararg limelightNames: String
    ){
        if (isReal()){
            for (llName in limelightNames){
                LimelightHelpers.setRobotOrientation(
                    llName,
                    heading.inUnit(degrees),
                    gyroscope.yawRate.inUnit(degrees/seconds),
                    gyroscope.pitch.inUnit(degrees),
                    0.0,
                    gyroscope.roll.inUnit(degrees),
                    0.0
                )
            }
        }
    }

    /**
     * The NavX firmware version.
     */
    public val firmwareVersion: String by logged{ ahrs.firmwareVersion }

    /**
     * Returns if the NavX is connected or not. Automatically false during simulation.
     */
    public val isConnected: Boolean by logged{ ahrs.isConnected && isReal() }

    /**
     * The heading of the NavX. Reports a value in between 0-360 degrees.
     */
    override val heading: Angle by logged{
        (if (isReal()) {
            // Negative sign because the navX reports clockwise as positive, whereas we want counterclockwise to be positive
            if (useFusedHeading && ahrs.isMagnetometerCalibrated && !ahrs.isMagneticDisturbance){
                -ahrs.fusedHeading.toDouble().ofUnit(degrees)
            }else{
                -ahrs.angle.ofUnit(degrees)
            }
        }else{
            IMUSimulation.getHeading()
        } + headingOffset).inputModulus(0.degrees..360.degrees)
    }

    /**
     * The gyroscope of the NavX. Implements [ThreeAxisGyroscope] and HeadingProvider.
     */
    public val gyroscope: Gyroscope = Gyroscope()

    /**
     * The accelerometer of the NavX. Implements [ThreeAxisAccelerometer].
     */
    public val accelerometer: Accelerometer = Accelerometer()


    public inner class Gyroscope internal constructor(): ThreeAxisGyroscope, Loggable {
        override val namespace = "NavX/Gyroscope"

        override val yaw: Angle by logged{
            if (isReal()) {
                ahrs.yaw.toDouble().ofUnit(degrees)
            } else {
                IMUSimulation.getHeading().inputModulus(-180.degrees..180.degrees)
            }
        }

        override val pitch: Angle by logged{
            if (isReal()) ahrs.pitch.toDouble().ofUnit(degrees) else Angle(0.0)
        }

        override val roll: Angle by logged{
            if (isReal()) ahrs.roll.toDouble().ofUnit(degrees) else Angle(0.0)
        }

        val yawRate: AngularVelocity by logged {
            if (isReal()) ahrs.rate.ofUnit(degrees/seconds) else AngularVelocity(0.0)
        }

        override val heading: Angle get() = this@ChargerNavX.heading

        fun zeroYaw(){
            ahrs.zeroYaw()
        }
    }


    public inner class Accelerometer internal constructor(): ThreeAxisAccelerometer, Loggable {
        override val namespace = "NavX/Accelerometer"
        private var previousXVelSim = Velocity(0.0)
        private var previousYVelSim = Velocity(0.0)

        override val xAcceleration: Acceleration by logged{
            if (isReal()){
                ahrs.worldLinearAccelX.toDouble().ofUnit(standardGravities)
            }else{
                val currXVelSim = IMUSimulation.getChassisSpeeds().xVelocity
                ((currXVelSim - previousXVelSim) / ChargerRobot.LOOP_PERIOD).also{
                    previousXVelSim = currXVelSim
                }
            }
        }

        override val yAcceleration: Acceleration by logged{
            if (isReal()){
                ahrs.worldLinearAccelY.toDouble().ofUnit(standardGravities)
            }else{
                val currYVelSim = IMUSimulation.getChassisSpeeds().yVelocity
                ((currYVelSim - previousYVelSim) / ChargerRobot.LOOP_PERIOD).also{
                    previousYVelSim = currYVelSim
                }
            }
        }

        override val zAcceleration: Acceleration by logged{
            if (isReal()) ahrs.worldLinearAccelZ.toDouble().ofUnit(standardGravities) else Acceleration(0.0)
        }
    }
}