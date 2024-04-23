@file:Suppress("RedundantVisibilityModifier", "unused", "KotlinConstantConditions", "LeakingThis")
package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * A base class for
 */
abstract class ChargerRobot(loopPeriod: Time = 0.02.seconds): TimedRobot(), Loggable {
    override val namespace = "RobotGeneral"

    public companion object: Loggable {
        override val namespace = "RobotGeneral"
        /**
         * Adds a specific function to the robot's periodic loop.
         *
         * All functions added this way will run before the command scheduler runs.
         */
        public fun runPeriodically(addToFront: Boolean = false, runnable: () -> Unit){
            if (addToFront){
                periodicRunnables.add(0,runnable)
            }else{
                periodicRunnables.add(runnable)
            }
        }

        /**
         * Adds a specific function to the robot's periodic loop, which runs after the robot's periodic loop ends.
         */
        public fun runPeriodicallyWithLowPriority(addToFront: Boolean = false, runnable: () -> Unit){
            if (addToFront){
                lowPriorityPeriodicRunnables.add(0,runnable)
            }else{
                lowPriorityPeriodicRunnables.add(runnable)
            }
        }

        /**
         * Removes a function from the periodic loop of the robot.
         */
        public fun removeFromLoop(runnable: () -> Unit){
            periodicRunnables.remove(runnable)
            lowPriorityPeriodicRunnables.remove(runnable)
        }

        /**
         * The loop period of the current robot.
         */
        public var LOOP_PERIOD: Time = 0.02.seconds
            private set

        /**
         * The [Field2d] that belongs to the robot.
         */
        public val FIELD: Field2d = Field2d().also{
            SmartDashboard.putData("Field", it)
        }

        /**
         * The current' year's apriltag field layout.
         */
        public val APRILTAG_LAYOUT: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()


        // stores alerts and periodic runnables
        private val periodicRunnables: MutableList<() -> Unit> = mutableListOf()
        private val lowPriorityPeriodicRunnables: MutableList<() -> Unit> = mutableListOf()
    }


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
    }


    override fun robotPeriodic() {
        periodicRunnables.forEach { it() }
        // Runs the Command Scheduler; polling buttons and scheduling commands.
        CommandScheduler.getInstance().run()
        lowPriorityPeriodicRunnables.forEach { it() }
    }
}