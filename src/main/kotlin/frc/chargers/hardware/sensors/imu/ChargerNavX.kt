@file:Suppress("unused")
package frc.chargers.hardware.sensors.imu

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.standardGravities
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.framework.logged


class ChargerNavX(
    val base: AHRS = AHRS(),
    private val useFusedHeading: Boolean = false,
    var simHeadingSource: () -> Angle = { Angle(0.0) }
): ZeroableHeadingProvider {
    private var headingOffset by logged(0.degrees)

    /**
     * Equivalent to the NavX [yaw].
     */
    override val heading: Angle get() = this.yaw

    /**
     * Zeroes the heading of the NavX.
     *
     * Does not zero the yaw; call navX.gyroscope.zeroYaw() if this is desired instead.
     */
    override fun zeroHeading(angle: Angle){
        if (isReal()){
            while (base.isCalibrating){
                println("Waiting for AHRS to calibrate...")
                // Wait for 1 second (hardware initialization) before zeroing heading
                Thread.sleep(1000)
            }

            println("AHRS has finished calibrating.")

            headingOffset = -heading + headingOffset + angle

            println("AHRS angle and fused heading has been offset. This does not impact ahrs.getYaw()!")
        }else{
            headingOffset = -simHeadingSource()
            println("AHRS heading has been offset in sim.")
        }
    }

    /**
     * The NavX firmware version.
     */
    val firmwareVersion: String by logged{ base.firmwareVersion }

    /**
     * Returns if the NavX is connected or not. Automatically false during simulation.
     */
    val isConnected: Boolean by logged{ base.isConnected && isReal() }

    /**
     * The heading of the NavX. Reports a continuous value(can go over 360 and -360 degrees).
     */
    val yaw: Angle by logged{
        if (isReal()) {
            // Negative sign because the navX reports clockwise as positive, whereas we want counterclockwise to be positive
            if (useFusedHeading && base.isMagnetometerCalibrated && !base.isMagneticDisturbance){
                -base.fusedHeading.toDouble().ofUnit(degrees)
            }else{
                -base.angle.ofUnit(degrees)
            }
        }else{
            simHeadingSource()
        } + headingOffset
    }

    /**
     * The NavX pitch. Is between -180 and 180 degrees.
     */
    val pitch: Angle by logged{
        if (isReal()) base.pitch.toDouble().ofUnit(degrees) else Angle(0.0)
    }

    /**
     * The NavX roll. Is between -180 and 180 degrees.
     */
    val roll: Angle by logged{
        if (isReal()) base.roll.toDouble().ofUnit(degrees) else Angle(0.0)
    }

    /**
     * The angular acceleration of the robot.
     */
    val yawRate: AngularVelocity by logged {
        base.rate.ofUnit(degrees/seconds)
    }

    val xAcceleration: Acceleration by logged{
        base.worldLinearAccelX.toDouble().ofUnit(standardGravities)
    }

    val yAcceleration: Acceleration by logged{
        base.worldLinearAccelY.toDouble().ofUnit(standardGravities)
    }

    val zAcceleration: Acceleration by logged{
        base.worldLinearAccelZ.toDouble().ofUnit(standardGravities)
    }
}