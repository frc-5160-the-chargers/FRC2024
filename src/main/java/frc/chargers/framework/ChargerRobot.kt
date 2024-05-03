@file:Suppress("RedundantVisibilityModifier", "unused", "KotlinConstantConditions", "LeakingThis")
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
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import java.lang.management.GarbageCollectorMXBean
import java.lang.management.ManagementFactory

/**
 * A base class for a generic Robot; extending [TimedRobot] and providing useful utilities.
 *
 * These include: global periodic function registration, automatic DataLogManager setup, and more.
 */
abstract class ChargerRobot(loopPeriod: Time = 0.02.seconds): TimedRobot(), Loggable, Tunable {
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
         * Adds a specific function to the robot's periodic loop, which runs at a higher frequency than normal.
         */
        fun runPeriodicHighFrequency(period: Time, runnable: () -> Unit){
            highFrequencyPeriodicRunnables.add(
                HighFrequencyPeriodicRunnable(period, runnable)
            )
        }

        /**
         * Adds a specific function to the robot's periodic loop, which runs after the robot's periodic loop ends.
         */
        fun runPeriodicLowPriority(runnable: () -> Unit){
            lowPriorityPeriodicRunnables.add(runnable)
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

    open val tuningMode: Boolean = false
    open val logFileFolder: String get() = if (isReal()) "/U/logs" else "logs"
    open val logFileName: String? = null
    open val logToFileOnly: Boolean = false

    init{
        val fileName = logFileName
        if (fileName == null){
            DataLogManager.start(logFileFolder)
        }else if (".wpilog" in fileName){
            DataLogManager.start(logFileFolder, fileName)
        }else{
            DataLogManager.start(logFileFolder, "$fileName.wpilog")
        }

        if (RobotBase.isReal()){
            DriverStation.startDataLog(DataLogManager.getLog())
        }

        log("loopPeriodSeconds", loopPeriod.inUnit(seconds))
        Tunable.tuningMode = tuningMode
        Loggable.fileOnly = logToFileOnly

        CommandScheduler.getInstance().apply{
            onCommandInitialize{
                log("ActiveCommands/${it.name}", true)
            }

            onCommandFinish {
                log("/ActiveCommands/${it.name}", false)
            }

            onCommandInterrupt {it ->
                log("/ActiveCommands/${it.name}", false)
            }
        }

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
        val eventLoop = EventLoop()
        eventLoop.poll()
    }


    override fun robotPeriodic() {
        logLatency("MeasuredLoopTime"){
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