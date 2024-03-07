@file:Suppress("RedundantVisibilityModifier", "unused", "KotlinConstantConditions")
package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.*
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.external.pathplanner.LocalADStarAK
import frc.chargers.constants.DashboardTuner
import frc.chargers.wpilibextensions.Alert
import frc.robot.BuildConstants
import org.littletonrobotics.junction.AutoLogOutputManager
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger.*
import org.littletonrobotics.junction.networktables.NT4Publisher
import java.io.File


/**
 * A base class for an FRC robot, acting as a wrapper around AdvantageKit's [LoggedRobot].
 *
 * This is intended to be used within the command-based framework.
 */
public open class ChargerRobot(
    private val getRobotContainer: () -> ChargerRobotContainer,
    private val config: RobotConfig
): LoggedRobot(config.loopPeriod.inUnit(seconds)){
    public companion object {
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
        public val FIELD: Field2d = Field2d()

        /**
         * The current' year's apriltag field layout.
         */
        public val APRILTAG_LAYOUT: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()



        // logging tables for custom loggable items
        internal var AK_LOGGABLE_REPLAY_TABLE: LogTable? = null
        internal var AK_LOGGABLE_REAL_TABLE: LogTable? = null
        @PublishedApi
        internal var hardwareConfigRetryLimit: Int = 1

        // stores alerts and periodic runnables
        private val periodicRunnables: MutableList<() -> Unit> = mutableListOf()
        private val lowPriorityPeriodicRunnables: MutableList<() -> Unit> = mutableListOf()
        private val noUsbSignalAlert = Alert.warning(text = "No logging to WPILOG is happening; cannot find USB stick")
    }

    private lateinit var robotContainer: ChargerRobotContainer
    private lateinit var autonomousCommand: Command
    private lateinit var testCommand: Command



    // Cancels a command; doing nothing if the command is not yet initialized.
    private inline fun cancelCommand(getCommand: () -> Command){
        try{
            val command = getCommand()
            command.cancel()
        }catch(_: UninitializedPropertyAccessException){}
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        try{
            hardwareConfigRetryLimit = config.hardwareConfigRetryLimit
            LOOP_PERIOD = config.loopPeriod
            configureAdvantageKit()

            DashboardTuner.tuningMode = config.tuningMode

            // inits robotContainer
            robotContainer = getRobotContainer()
            robotContainer.robotInit()

            // adds package directories for auto log output
            AutoLogOutputManager.addPackage("frc.chargers")
            AutoLogOutputManager.addPackage("frc.robot")

            Pathfinding.setPathfinder(LocalADStarAK())

            SmartDashboard.putData("Field", FIELD)

            CommandScheduler.getInstance().apply{
                onCommandInitialize{
                    recordOutput("/ActiveCommands/${it.name}", true)
                }

                onCommandFinish {
                    recordOutput("/ActiveCommands/${it.name}", false)
                }

                onCommandInterrupt {it ->
                    recordOutput("/ActiveCommands/${it.name}", false)
                }
            }
        }catch(e: Exception){
            println("Error has been caught in [robotInit].")
            config.onError(e)
            throw e
        }

    }

    private fun configureAdvantageKit(){
        setUseTiming(
            RobotBase.isReal() || !config.isReplay
        )
        recordMetadata(
            "Robot", if (RobotBase.isReal()) "REAL" else if (config.isReplay) "REPLAY" else "SIM"
        )
        recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
        when(BuildConstants.DIRTY){
            0 -> recordMetadata("GitDirty", "All changes committed")
            1 -> recordMetadata("GitDirty", "Uncommitted changes")
            else -> recordMetadata("GitDirty", "Unknown")
        }




        if (RobotBase.isReal()){
            // real robot
            val availablePaths = File("/media/").listFiles()?.map{ it.path } ?: listOf()
            recordMetadata("All File Paths Available", availablePaths.toString())
            var noLogHappening = true
            for (filePathSuffix in config.filePathOptions){
                val filePathOption = "/media/$filePathSuffix"
                if (filePathOption in availablePaths){
                    addDataReceiver(WPILOGWriter(filePathOption))
                    noLogHappening = false
                    break
                }
            }
            if (noLogHappening){
                noUsbSignalAlert.active = true
            }
        }else if (config.isReplay){
            // replay mode; sim
            val path = config.logFilePath ?: LogFileUtil.findReplayLog()
            setReplaySource(WPILOGReader(path))
            addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")))
        }

        if (config.logToNetworkTables && !config.isReplay){
            addDataReceiver(NT4Publisher())
        }

        config.extraLoggerConfig()

        // no more configuration from this point on
        start()

        // configuration for AdvantageKitLoggable
        val baseEntry = LogTable(0)
        AK_LOGGABLE_REPLAY_TABLE = baseEntry.getSubtable("ReplayOutputs")
        AK_LOGGABLE_REAL_TABLE = baseEntry.getSubtable("RealOutputs")
    }

    private val gcTimer = Timer()
    private fun runTopPriorityPeriodicFunctions(){
        recordLatency("LoggedRobot/PeriodicRunnableLoopTime/RegularPriority"){
            periodicRunnables.forEach{
                it()
            }
        }
    }

    private fun runLowPriorityPeriodicFunctions(){
        recordLatency("LoggedRobot/PeriodicRunnableLoopTime/LowPriority"){
            lowPriorityPeriodicRunnables.forEach{
                it()
            }
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        try{
            if (gcTimer.advanceIfElapsed(5.0)){
                System.gc()
            }

            runTopPriorityPeriodicFunctions()
            // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
            // commands, running already-scheduled commands, removing finished or interrupted commands,
            // and running subsystem periodic() methods.  This must be called from the robot's periodic
            // block in order for anything in the Command-based framework to work.
            CommandScheduler.getInstance().run()
            runLowPriorityPeriodicFunctions()
        }catch(e: Exception){
            println("Error has been caught in [robotPeriodic].")
            config.onError(e)
            throw e
        }

    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
        try{
            robotContainer.disabledInit()
        }catch(e: Exception){
            println("Error has been caught in [disabledInit].")
            config.onError(e)
            throw e
        }
    }

    override fun disabledPeriodic() {
        try{
            robotContainer.disabledPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [disabledPeriodic].")
            config.onError(e)
            throw e
        }
    }

    /** This autonomous runs the autonomous command selected by your RobotContainer class.  */
    override fun autonomousInit() {
        try{
            robotContainer.autonomousInit()
            cancelCommand{ testCommand }
            autonomousCommand = robotContainer.autonomousCommand
            autonomousCommand.schedule()
        }catch(e: Exception){
            println("Error has been caught in [autonomousInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
        try{
            robotContainer.autonomousPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [autonomousPeriodic].")
            config.onError(e)
            throw e
        }
    }

    override fun teleopInit() {
        try{
            robotContainer.teleopInit()
            cancelCommand{ autonomousCommand }
            cancelCommand{ testCommand }
        }catch(e: Exception){
            println("Error has been caught in [teleopInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {
        try{
            robotContainer.teleopPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [teleopPeriodic].")
            config.onError(e)
            throw e
        }
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        try{
            CommandScheduler.getInstance().cancelAll()
            testCommand = robotContainer.testCommand
            robotContainer.testInit()
            testCommand.schedule()
        }catch(e: Exception){
            println("Error has been caught in [testInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {
        try{
            robotContainer.testPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [testPeriodic].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {
        try{
            robotContainer.simulationInit()
        }catch(e: Exception){
            println("Error has been caught in [simulationInit].")
            config.onError(e)
            throw e
        }
    }

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
        try{
            robotContainer.simulationPeriodic()
        }catch(e: Exception){
            println("Error has been caught in [simulationPeriodic].")
            config.onError(e)
            throw e
        }
    }
}









