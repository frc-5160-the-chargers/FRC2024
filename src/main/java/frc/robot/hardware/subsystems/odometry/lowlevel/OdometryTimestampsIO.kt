package frc.robot.hardware.subsystems.odometry.lowlevel

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.robot.hardware.subsystems.odometry.OdometryThread

/**
 * Handles the timestamps required for multi-threaded odometry.
 */
class OdometryTimestampsIO {
    private val logInputs = LoggableInputsProvider(
        "MultiThreadedOdometry",
        runBeforeInputUpdate = OdometryThread.ODOMETRY_LOCK::lock,
        runAfterInputUpdate = OdometryThread.ODOMETRY_LOCK::unlock
    )
    private val timestampsQueue = OdometryThread.getInstance().makeTimestampQueue()

    val timestamps by logInputs.quantityList{
        timestampsQueue
            .stream()
            .map{ it.ofUnit(seconds) }
            .toList()
            .also{ timestampsQueue.clear() }
    }
}