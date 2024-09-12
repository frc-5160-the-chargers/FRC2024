@file:Suppress("unused", "KotlinConstantConditions", "LeakingThis")
package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import kcommand.LoggedCommand
import org.littletonrobotics.urcl.URCL
import java.io.File
import java.lang.management.GarbageCollectorMXBean
import java.lang.management.ManagementFactory

/**
 * A base class for a generic Robot; extending [TimedRobot] and providing useful utilities.
 *
 * These include: global periodic function registration, automatic DataLogManager setup,
 * automatic URCL(Unofficial REV-Compatible logger) startup, and more.
 */
abstract class ChargerRobot(
    loopPeriod: Time = 0.02.seconds,
    tuningMode: Boolean = false,
    logFileFolder: String = if (isReal()) "/U/logs" else "logs",
    logFileName: String? = null,
    useURCL: Boolean = true
): TimedRobot(), Loggable, Tunable {
    override val namespace = "RobotGeneral"

    companion object: Loggable {
        override val namespace = "RobotGeneral"

        private class HighFrequencyPeriodicRunnable(
            val period: Time,
            val toRun: () -> Unit
        )

        private val periodicRunnables = mutableListOf<() -> Unit>()
        private val highFrequencyPeriodicRunnables = mutableListOf<HighFrequencyPeriodicRunnable>()
        private val lowPriorityPeriodicRunnables = mutableListOf<() -> Unit>()

        /**
         * Adds a specific function to the robot's periodic loop.
         *
         * All functions added this way will run before the command scheduler runs.
         */
        fun runPeriodic(runnable: () -> Unit){
            periodicRunnables.add(runnable)
        }

        /**
         * Adds a specific function to the robot's periodic loop, which runs after the robot's periodic loop ends.
         */
        fun runPeriodicWithLowPriority(runnable: () -> Unit){
            lowPriorityPeriodicRunnables.add(runnable)
        }

        /**
         * Adds a specific function to the robot's periodic loop, which can run in a higher frequency
         * than normal.
         *
         * Equivalent to WPILib's [addPeriodic] method.
         */
        fun runPeriodicAtPeriod(period: Time, runnable: () -> Unit){
            highFrequencyPeriodicRunnables.add(
                HighFrequencyPeriodicRunnable(period, runnable)
            )
        }

        /**
         * The loop period of the current robot.
         */
        var LOOP_PERIOD: Time = 0.02.seconds
            private set

        /**
         * The [Field2d] that belongs to the robot.
         */
        val FIELD: Field2d = Field2d().also{
            SmartDashboard.putData("Field", it)
        }
    }

    // Credits: 6328
    // logger utilities for measuring java garbage collection
    private val gcBeans: List<GarbageCollectorMXBean> = ManagementFactory.getGarbageCollectorMXBeans()
    private val lastTimes = LongArray(gcBeans.size)
    private val lastCounts = LongArray(gcBeans.size)
    private fun logGcData(){
        var accumTime: Long = 0
        var accumCounts: Long = 0
        for (i in gcBeans.indices) {
            val gcTime = gcBeans[i].collectionTime
            val gcCount = gcBeans[i].collectionCount
            accumTime += gcTime - lastTimes[i]
            accumCounts += gcCount - lastCounts[i]

            lastTimes[i] = gcTime
            lastCounts[i] = gcCount
        }

        log("GCTimeMS", accumTime.toDouble())
        log("GCCounts", accumCounts.toDouble())
    }

    private val commandScheduler = CommandScheduler.getInstance()

    init{
        LoggedCommand.configure(
            logCommandRunning = this::log,
            logCommandExecutionTime = this::log
        )

        if (logFileName == null) {
            DataLogManager.start(logFileFolder)
        } else {
            val actualFileName = if (logFileName.endsWith(".wpilog")) logFileName else "$logFileName.wpilog"
            File("$logFileFolder/$actualFileName").createNewFile()
            DataLogManager.start(logFileFolder, actualFileName)
        }

        if (RobotBase.isReal()){
            DriverStation.startDataLog(DataLogManager.getLog())
            if (useURCL){
                URCL.start()
            }
        }

        log("loopPeriodSeconds", loopPeriod.inUnit(seconds))
        Tunable.tuningMode = tuningMode

        Pathfinding.setPathfinder(LocalADStar())

        var currPose = Pose2d()

        log("Pathplanner/deviationFromTargetPose/xMeters", 0.0)
        log("Pathplanner/deviationFromTargetPose/yMeters", 0.0)
        log("Pathplanner/deviationFromTargetPose/rotationRad", 0.0)

        PathPlannerLogging.setLogCurrentPoseCallback { currPose = it }

        PathPlannerLogging.setLogTargetPoseCallback {
            log("Pathplanner/deviationFromTargetPose/xMeters", it.x - currPose.x)
            log("Pathplanner/deviationFromTargetPose/yMeters", it.y - currPose.y)
            log("Pathplanner/deviationFromTargetPose/rotationRad", (it.rotation - currPose.rotation).radians)
        }

        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)
    }

    override fun robotPeriodic() {
        logLatency("ChargerRobotLoopTime"){
            if (highFrequencyPeriodicRunnables.size > 0){
                for (runnable in highFrequencyPeriodicRunnables){
                    addPeriodic(
                        runnable.toRun,
                        runnable.period.inUnit(seconds),
                        0.005
                    )
                }
                highFrequencyPeriodicRunnables.clear()
            }

            periodicRunnables.forEach{ it() }
            // Runs the Command Scheduler; polling buttons and scheduling commands.
            commandScheduler.run()
            lowPriorityPeriodicRunnables.forEach { it() }

            logGcData()
        }
    }
}