package frc.robot.hardware.subsystems.odometry.lowlevel

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.utils.math.equations.epsilonEquals
import frc.robot.ODOMETRY_UPDATE_FREQUENCY_HZ
import frc.robot.hardware.subsystems.odometry.OdometryThread
import java.util.*

/**
 * Handles gyro readings for multi-threaded odometry.
 */
class GyroOdometryIO(navX: ChargerNavX) {
    init{
        require(navX.ahrs.requestedUpdateRate.toDouble() epsilonEquals ODOMETRY_UPDATE_FREQUENCY_HZ){
            "The NavX update rate is incorrect. Current update rate: " + navX.ahrs.requestedUpdateRate.toDouble() + ", Needed rate: " + ODOMETRY_UPDATE_FREQUENCY_HZ
        }
    }

    // handles logging and replay for gyro odometry
    private val logInputs = LoggableInputsProvider(
        "MultiThreadedOdometry/gyro",
        runBeforeInputUpdate = OdometryThread.ODOMETRY_LOCK::lock,
        runAfterInputUpdate = OdometryThread.ODOMETRY_LOCK::unlock
    )

    // degrees
    private val gyroReadingsQueue = OdometryThread.getInstance().registerSignal{ OptionalDouble.of(navX.ahrs.angle) }

    val gyroReadings by logInputs.quantityList{
        gyroReadingsQueue
            .stream()
            .map { it.ofUnit(degrees) - navX.getHeadingOffset() }
            .toList()
            .also{ gyroReadingsQueue.clear() }
    }
}