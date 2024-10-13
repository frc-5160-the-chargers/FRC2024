package frc.robot.rigatoni

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.controls.PIDController
import kcommand.InstantCommand
import kcommand.RunCommand
import kcommand.commandbuilder.buildCommand
import kcommand.setDefaultRunCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog
import frc.chargers.framework.Tunable
import frc.chargers.framework.tunable
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.utils.squareMagnitude
import frc.chargers.wpilibextensions.distanceTo
import frc.chargers.wpilibextensions.flipWhenRedAlliance
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.wpilibextensions.onDoubleClick
import frc.robot.rigatoni.subsystems.*
import frc.robot.rigatoni.subsystems.shooter.Shooter
import kotlin.math.abs
import kotlin.math.max

/**
 * Our competition robot; rigatoni.
 */
@Suppress("unused")
class CompetitionRobot: ChargerRobot() {
    private val gyro = ChargerNavX()
    private val drivetrain = getDrivetrain(gyro)
    private val pivot = Pivot(disable = true)
    private val groundIntake = GroundIntakeSerializer(disable = true)
    private val shooter = Shooter(disable = true)
    private val climber = Climber()
    private val noteObserver = NoteObserver()
    
    private val driverController = DriverController(DRIVER_CONTROLLER_PORT, "DriverController")
    private val driverTouchpad = driverController.touchpad()
    private val operatorController = CommandXboxController(OPERATOR_CONTROLLER_PORT)

    private val autoChooser = SendableChooser<Command>()

    val testTunable by tunable(2.0, "SomeValue").onChange { println("Hi!") }

    init {
        DriverStation.silenceJoystickConnectionWarning(true)
        PortForwarder.add(5800, "photonvision.local", 5800)
        gyro.simHeadingSource = { drivetrain.calculatedHeading }
        HorseLog.setOptions(
            HorseLog.getOptions()
                .withNtPublish(true)
                .withCaptureDs(true)
                .withLogExtras(true)
                .withLogEntryQueueCapacity(3000)
        )
        HorseLog.setPdh(PowerDistribution(1, PowerDistribution.ModuleType.kRev))
        Trigger(DriverStation::isFMSAttached).onTrue(
            InstantCommand { HorseLog.setOptions(HorseLog.getOptions().withNtPublish(false)) }
        )

        setDefaultCommands()
        setButtonBindings()

        autoChooser.setDefaultOption(
            "Taxi",
            RunCommand(drivetrain){ drivetrain.swerveDrive(0.2, 0.0, 0.0, fieldRelative = false) }
                .withTimeout(5.0)
        )
        // getAutoCommands is implemented below
        for (autoCommand in getAutoCommands()){
            autoChooser.addOption(autoCommand.name, autoCommand)
        }
        SmartDashboard.putData("AutoChoices", autoChooser)
        Tunable.tuningMode = true
    }

    override fun autonomousInit() {
        autoChooser.selected?.schedule()
    }

    override fun autonomousExit() {
        autoChooser.selected?.cancel()
    }

    private fun setButtonBindings() {
        driverController.apply {
            val aimTriggers: List<Trigger>
            val noteIntakeTrigger: Trigger
            val climbUpTrigger: Trigger
            val climbDownTrigger: Trigger
            if (DRIVER_RIGHT_HANDED){
                aimTriggers = listOf(povUp(), povRight(), povDown(), povLeft())
                noteIntakeTrigger = L2()
                climbUpTrigger = triangle()
                climbDownTrigger = cross()
            } else {
                aimTriggers = listOf(triangle(), circle(), cross(), square())
                noteIntakeTrigger = R2()
                climbUpTrigger = povUp()
                climbDownTrigger = povDown()
            }

            touchpad().onDoubleClick(InstantCommand{ gyro.zeroHeading(180.degrees) })

            var current = 0.degrees
            for (trigger in aimTriggers){
                trigger.whileTrue(AngleAimCommand(current, drivetrain) { driverController.swerveOutput })
                current -= 90.degrees
            }

            noteIntakeTrigger.whileTrue(
                noteIntakeDriverAssist()
                    .alongWith(RunCommand(groundIntake){ groundIntake.intake() })
            )

            climbUpTrigger.whileTrue(
                RunCommand(climber){
                    climber.moveLeftHook(1.0)
                    climber.moveRightHook(1.0)
                }
            )

            climbDownTrigger.whileTrue(
                RunCommand(climber){
                    climber.moveLeftHook(-1.0)
                    climber.moveRightHook(-1.0)
                }
            )
        }

        operatorController.apply {
            // interrupt behavior set as to prevent command scheduling conflicts
            a().whileTrue(
                pivotAngleCommand(PivotAngle.AMP)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )
            b().whileTrue(pivotAngleCommand(PivotAngle.STOWED))
            x().whileTrue(
                pivotAngleCommand(PivotAngle.SOURCE)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            rightTrigger().whileTrue(
                RunCommand(groundIntake){ groundIntake.intake() }
            )
            leftTrigger().whileTrue(
                RunCommand(groundIntake, shooter){ // ground outtake command
                    groundIntake.outtake()
                    shooter.setVoltage((-6).volts)
                }
            )

            rightBumper().and(leftBumper().negate()).whileTrue(passNoteToShooter())
            leftBumper().and(rightBumper().negate()).whileTrue(
                RunCommand(shooter, pivot){ // spinup shooter command
                    shooter.outtakeAtSpeakerSpeed()
                    pivot.setAngle(PivotAngle.SPEAKER)
                }
            )
            leftBumper().and(rightBumper()).whileTrue(
                shootInSpeaker(shooterSpinUpTime = 0.3.seconds)
            )

            start().onDoubleClick(
                InstantCommand { drivetrain.reSyncRelativeEncoders() }
            )
        }
    }

    private fun setDefaultCommands(){
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(driverController.swerveOutput, fieldRelative = !driverTouchpad.asBoolean)
        }

        shooter.setDefaultRunCommand{
            var speed = applyDeadband(operatorController.leftY, SHOOTER_DEADBAND)
            speed *= SHOOTER_SPEED_MULTIPLIER
            HorseLog.log("OperatorController/ShooterSpeed", speed)
            if (speed > 0.0 && noteObserver.noteInRobot){
                shooter.setIdle()
            }else{
                shooter.setSpeed(speed)
            }
        }

        pivot.defaultCommand = RunCommand(pivot){
            var speed = applyDeadband(operatorController.rightY, PIVOT_DEADBAND).squareMagnitude()
            speed *= PIVOT_SPEED_MULTIPLIER
            HorseLog.log("OperatorController/PivotSpeed", speed)
            pivot.setSpeed(speed)
        }.finallyDo(pivot::setIdle)

        climber.setDefaultRunCommand{ climber.setIdle() }

        groundIntake.setDefaultRunCommand { groundIntake.setIdle() }
    }

    private fun shouldPathFind(path: PathPlannerPath): Boolean {
        val pathStartPose = path.previewStartingHolonomicPose.flipWhenRedAlliance()
        val pathEndPose = path.pathPoses.last().flipWhenRedAlliance()
        val drivetrainPose = drivetrain.robotPose

        return drivetrainPose.distanceTo(pathStartPose) > ACCEPTABLE_DISTANCE_BEFORE_PATHFIND &&
                drivetrainPose.distanceTo(pathEndPose) > drivetrainPose.distanceTo(pathStartPose)
    }

    private fun shouldStartNotePursuit(): Boolean {
        val currentState = noteObserver.state
        return currentState is NoteState.Detected &&
                currentState.distanceToNote <= ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
    }

    /* Commands */

    private fun noteIntakeDriverAssist(
        getChassisPowers: () -> ChassisPowers = { driverController.swerveOutput }
    ) = buildCommand("NoteIntakeDriverAssist") {
        require(drivetrain)

        fun getNotePursuitSpeed(txValue: Double): Double {
            val swerveOutput = getChassisPowers()
            return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - txValue / 50.0) // scales based off of the vision target error
        }

        // custom PID controller constructor that accepts pid constants
        val aimController by getOnceDuringRun { PIDController(AIM_TO_NOTE_PID) }

        // regular drive occurs until suitable target found
        loopUntil(::shouldStartNotePursuit){
            drivetrain.swerveDrive(driverController.swerveOutput, fieldRelative = !driverTouchpad.asBoolean)
        }

        // drives back to grab note
        loop {
            val forwardPower: Double
            val rotationPower: Double
            val state = noteObserver.state
            if (state is NoteState.Detected){
                forwardPower = getNotePursuitSpeed(state.tx)
                rotationPower = aimController.calculate(state.tx, 0.0)
            } else {
                forwardPower = getNotePursuitSpeed(0.0)
                rotationPower = 0.0
            }
            drivetrain.swerveDrive(forwardPower, 0.0, rotationPower, fieldRelative = false)
        }

        onEnd {
            drivetrain.stop()
        }
    }

    // Note: pivot.setAngle() is the method call,
    // while pivotAngleCommand() is a command that ends by itself.
    private fun pivotAngleCommand(target: Angle) =
        buildCommand("PivotAngleCommand") {
            require(pivot)

            runOnce{ pivot.setAngle(target) }

            loopUntil({ pivot.atTarget }){ pivot.setAngle(target) }

            onEnd{ pivot.setIdle() }
        }

    private fun passNoteToShooter() = buildCommand("PassNoteToShooter") {
        require(groundIntake, shooter)

        realRobotOnly {
            loopUntil({noteObserver.state == NoteState.InShooter}){
                shooter.receiveFromGroundIntake()
                groundIntake.passToShooterSlow()
            }
        }

        loopForDuration(0.4){
            shooter.receiveFromGroundIntake()
            groundIntake.passToShooterSlow()
        }

        onEnd{
            groundIntake.setIdle()
            shooter.setIdle()
        }
    }

    private fun shootInAmp() = buildCommand("ShootInAmp") {
        require(noteObserver, shooter, pivot)

        +pivotAngleCommand(PivotAngle.AMP)

        realRobotOnly {
            loopUntil({noteObserver.state == NoteState.InShooter}){
                shooter.outtakeAtAmpSpeed()
            }

            loopUntil({noteObserver.state == NoteState.None}){
                shooter.outtakeAtAmpSpeed()
            }
        }

        loopForDuration(0.3){
            shooter.outtakeAtAmpSpeed()
        }
    }.withTimeout(4.0)

    /**
     * Accelerates a note out of the serializer and through the shooter.
     * Presumes that the shooter is already spinning.
     */
    private fun runNoteThroughShooter() = buildCommand {
        require(groundIntake)

        realRobotOnly {
            loopUntil({noteObserver.state == NoteState.InShooter}) { groundIntake.passToShooterFast() }

            loopUntil({noteObserver.state == NoteState.None}) { groundIntake.passToShooterFast() }
        }

        loopForDuration(0.4){ groundIntake.passToShooterFast() }

        onEnd { groundIntake.setIdle() }
    }

    /**
     * Runs the full shooting sequence.
     */
    private fun shootInSpeaker(shooterSpinUpTime: Time = 1.seconds, movePivot: Boolean = true) =
        buildCommand("ShootInSpeaker") {
            require(shooter, groundIntake, pivot)

            parallel { // since there are 2 deadlines, the loop {} block will stop when both deadlines end
                wait(shooterSpinUpTime.inUnit(seconds)).asDeadline()
                waitUntil { !movePivot || pivot.atTarget }.asDeadline()
                loop{
                    shooter.outtakeAtSpeakerSpeed()
                    if (movePivot) pivot.setAngle(PivotAngle.SPEAKER)
                }
            }

            +runNoteThroughShooter() // shooter is still running at this point

            onEnd {
                shooter.setIdle()
                groundIntake.setIdle()
                pivot.setIdle()
            }
        }

    /* Auto Components */

    private fun getAutoCommands() =
        listOf(
            buildCommand("1 Piece Amp + Taxi"){
                +ampAutoStartup()
                +pivotAngleCommand(PivotAngle.STOWED)
                +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxiShort"))
            },

            buildCommand("2-3 Piece Amp"){
                +ampAutoStartup()
                +driveAndIntake(
                    PathPlannerPath.fromPathFile("AmpGrabG1"),
                    movePivot = true,
                    intakeSpinupTime = 0.5.seconds,
                    noteIntakeTime = 0.5.seconds
                )
                +driveThenScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG1"))
                +driveAndIntake(
                    PathPlannerPath.fromPathFile("AmpGrabG2"),
                    movePivot = true,
                    noteIntakeTime = 0.5.seconds
                )
                +driveThenScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG2"))
            },

            speakerAutoStartup(AutoStartingPose::getSpeakerLeft).withName("1 Piece Speaker"),

            buildCommand("1 Piece Speaker + Taxi"){
                +speakerAutoStartup(AutoStartingPose::getSpeakerLeft)

                loopForDuration(5){
                    drivetrain.swerveDrive(0.2,0.0,0.0)
                }

                onEnd{ drivetrain.stop() }
            },

            buildCommand("4.5 Piece Speaker"){
                +speakerAutoStartup(AutoStartingPose::getSpeakerCenter)
                repeat(3) { i ->
                    +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 1}"))
                    +driveThenShoot(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 2}"))
                }
                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.7"))
            },

            buildCommand("4.5 Piece Speaker(Continuous Shooter Running)") {
                +speakerAutoStartup(AutoStartingPose::getSpeakerCenter)
                parallelRace {
                    runSequence {
                        repeat(3) { i ->
                            +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 1}"))
                            +AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 2}"))
                            +runNoteThroughShooter()
                        }
                    }

                    loop { // will never finish; simply runs in background
                        shooter.outtakeAtSpeakerSpeed()
                        pivot.setAngle(PivotAngle.SPEAKER)
                    }
                }
                runOnce { shooter.setIdle() }
                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.7"))
            }
        )

    private fun ampAutoStartup() = buildCommand("AmpAutoStartup") {
        require(drivetrain, pivot, shooter, groundIntake)

        parallel {
            runSequence {
                runOnce { drivetrain.resetPose(AutoStartingPose.getAmp()) }
                wait(0.3)
                +AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveToAmp"))
            }

            +pivotAngleCommand(PivotAngle.AMP)
        }

        +shootInAmp()
    }

    private fun speakerAutoStartup(getStartingPose: () -> Pose2d) = buildCommand("SpeakerAutoStartup") {
        require(drivetrain, pivot, shooter, groundIntake)

        runOnce{ drivetrain.resetPose(getStartingPose()) }

        +shootInSpeaker(shooterSpinUpTime = 1.5.seconds)
    }

    private fun driveAndIntake(
        path: PathPlannerPath,
        movePivot: Boolean = false,
        useGroundIntakeSensor: Boolean = true,
        intakeSpinupTime: Time = 0.seconds,
        noteIntakeTime: Time = 0.seconds,
    ) = buildCommand("DriveAndIntake", log = true){
        require(drivetrain, pivot, groundIntake)

        loopForDuration(intakeSpinupTime.inUnit(seconds)){
            groundIntake.intake()
        }

        parallelRace {
            runSequence {
                +(AutoBuilder.followPath(path).until(::shouldStartNotePursuit))

                +(noteIntakeDriverAssist{ ChassisPowers(0.3, 0.0, 0.0) }
                    .onlyWhile{ noteObserver.hasCamera && !noteObserver.noteFound })

                runOnce { drivetrain.stop() }

                wait(noteIntakeTime.inUnit(seconds))
            }

            waitUntil { // if using ground intake sensor, stop intaking + path when note detected
                useGroundIntakeSensor &&
                noteObserver.hasGroundIntakeSensor &&
                noteObserver.state != NoteState.InSerializer
            }

            loop {
                groundIntake.intake()
                if (movePivot) pivot.setAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
            }
        }

        onEnd {
            drivetrain.stop()
            groundIntake.setIdle()
            pivot.setIdle()
        }
    }

    private fun driveThenScoreAmp(path: PathPlannerPath) = buildCommand("DriveThenScoreAmp") {
        require(drivetrain, pivot, shooter, groundIntake)

        parallel {
            +AutoBuilder.followPath(path)

            runSequence {
                +pivotAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF) // insurance in case the pivot isn't already in the right position
                +passNoteToShooter()
            }
        }

        +shootInAmp()
    }

    private fun driveThenShoot(
        path: PathPlannerPath,
        shooterSpinUpTime: Time = 0.seconds
    ) = buildCommand("DriveThenShoot(Speaker)") {
        require(drivetrain, pivot, shooter, groundIntake)

        parallel {
            +AutoBuilder.followPath(path).asDeadline()

            loop{
                shooter.outtakeAtSpeakerSpeed()
                groundIntake.setIdle()
                pivot.setAngle(PivotAngle.SPEAKER)
            }
        }

        +shootInSpeaker(shooterSpinUpTime)
    }
}