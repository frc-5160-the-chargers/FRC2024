@file:Suppress("RedundantVisibilityModifier", "unused", "KotlinConstantConditions", "LeakingThis")
package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.chargers.constants.DashboardTuner


abstract class ChargerRobot(loopPeriod: Time = 0.02.seconds): TimedRobot(loopPeriod.inUnit(seconds)), Loggable {

    override val logGroup = "RobotGeneral"

    public companion object: Loggable {
        override val logGroup = "RobotGeneral"
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


    open var tuningMode: Boolean = false
        set(value){
            log("tuningMode", value)
            field = value
            DashboardTuner.tuningMode = value
        }

    open val logFileFolder: String? = null
    open val logFileName: String? = null

    open val logToFileOnly: Boolean = false


    init{
        DataLogManager.logNetworkTables(!logToFileOnly)
        if (logFileFolder != null){
            if (logFileName != null){
                DataLogManager.start(logFileFolder)
            }else{
                DataLogManager.start(logFileFolder, logFileName)
            }
        }
        log("loopPeriodSeconds", loopPeriod.inUnit(seconds))



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

        var currPose = Pose2d()

        log(Pose2d.struct, "Pathplanner/currentPose", Pose2d())
        log(Pose2d.struct, "Pathplanner/targetPose", Pose2d())
        log("Pathplanner/deviationFromTargetPose/xMeters", 0.0)
        log("Pathplanner/deviationFromTargetPose/yMeters", 0.0)
        log("Pathplanner/deviationFromTargetPose/rotationRad", 0.0)

        PathPlannerLogging.setLogCurrentPoseCallback {
            currPose = it
            log(Pose2d.struct, "Pathplanner/currentPose", it)
        }

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback {
            log(Pose2d.struct, "Pathplanner/targetPose", it)
            log(
                "Pathplanner/deviationFromTargetPose/xMeters",
                it.x - currPose.x
            )
            log(
                "Pathplanner/deviationFromTargetPose/yMeters",
                it.y - currPose.y
            )
            log(
                "Pathplanner/deviationFromTargetPose/rotationRad",
                (it.rotation - currPose.rotation).radians
            )
        }
    }


    override fun robotPeriodic() {
        periodicRunnables.forEach { it() }
        // Runs the Command Scheduler; polling buttons and scheduling commands.
        CommandScheduler.getInstance().run()
        lowPriorityPeriodicRunnables.forEach { it() }
    }
}