package frc.robot.rigatoni

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.math.geometry.Pose2d
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
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.utils.squareMagnitude
import frc.chargers.wpilibextensions.distanceTo
import frc.chargers.wpilibextensions.flipWhenRedAlliance
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.wpilibextensions.onDoubleClick
import frc.robot.rigatoni.inputdevices.PS5SwerveController
import frc.robot.rigatoni.subsystems.Climber
import frc.robot.rigatoni.subsystems.GroundIntakeSerializer
import frc.robot.rigatoni.subsystems.NoteObserver
import frc.robot.rigatoni.subsystems.getDrivetrain
import frc.robot.rigatoni.subsystems.Pivot
import frc.robot.rigatoni.subsystems.PivotAngle
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
    private val pivot = Pivot()
    private val groundIntake = GroundIntakeSerializer()
    private val shooter = Shooter()
    private val climber = Climber()
    private val noteObserver = NoteObserver()
    
    private val driverController = PS5SwerveController(DRIVER_CONTROLLER_PORT, DEFAULT_DEADBAND, DRIVER_RIGHT_HANDED)
    private val driverTouchpad = driverController.touchpad()
    private val operatorController = CommandXboxController(OPERATOR_CONTROLLER_PORT)

    private val autoChooser = SendableChooser<Command>()

    override fun robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true)
        gyro.simHeadingSource = { drivetrain.heading }
        SmartDashboard.putData(
            "Power Distribution",
            PowerDistribution(1, PowerDistribution.ModuleType.kRev)
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
                trigger.whileTrue(AngleAimCommand({current}, drivetrain, { driverController.swerveOutput }))
                current -= 90.degrees
            }

            noteIntakeTrigger.whileTrue(noteIntakeDriverAssist())

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
                setPivotAngle(PivotAngle.AMP)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )
            b().whileTrue(setPivotAngle(PivotAngle.STOWED))
            x().whileTrue(
                setPivotAngle(PivotAngle.SOURCE)
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

            rightBumper().and(leftBumper().negate()).whileTrue(passSerializedNote())
            leftBumper().and(rightBumper().negate()).whileTrue(
                RunCommand(shooter, pivot){ // spinup shooter command
                    shooter.outtakeAtSpeakerSpeed()
                    pivot.setAngle(PivotAngle.SPEAKER)
                }
            )
            leftBumper().and(rightBumper()).whileTrue(
                shootInSpeaker(shooterSpinUpTime = 0.3.seconds)
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
            log("OperatorController/ShooterSpeed", speed)
            if (speed > 0.0 && noteObserver.noteInRobot){
                shooter.setIdle()
            }else{
                shooter.setSpeed(speed)
            }
        }

        pivot.defaultCommand = RunCommand(pivot){
            var speed = applyDeadband(operatorController.rightY, PIVOT_DEADBAND).squareMagnitude()
            speed *= PIVOT_SPEED_MULTIPLIER
            log("OperatorController/PivotSpeed", speed)
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

    /* Commands */

    private fun followPathOptimal(path: PathPlannerPath) = buildCommand(name = "FollowPathOptimal"){
        runSequenceIf({shouldPathFind(path)}){
            +AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS)
        }.orElse{
            +AutoBuilder.followPath(path)
        }
    }

    private fun noteIntakeDriverAssist(
        getChassisPowers: () -> ChassisPowers = { driverController.swerveOutput }
    ) = buildCommand {
        require(drivetrain)

        fun getNotePursuitSpeed(txValue: Double): Double {
            val swerveOutput = getChassisPowers()
            return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - txValue / 50.0) // scales based off of the vision target error
        }

        fun shouldStartPursuit(): Boolean {
            val currentState = noteObserver.state
            return currentState is NoteObserver.State.NoteDetected &&
                    currentState.distanceToNote <= ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
        }

        // custom PID controller constructor that accepts pid constants
        val aimController by getOnceDuringRun { PIDController(AIM_TO_NOTE_PID) }

        // regular drive occurs until suitable target found
        loopUntil(::shouldStartPursuit){
            drivetrain.swerveDrive(driverController.swerveOutput, fieldRelative = !driverTouchpad.asBoolean)
        }

        // drives back to grab note
        loop {
            val state = noteObserver.state
            if (state is NoteObserver.State.NoteDetected){
                val forwardPower = getNotePursuitSpeed(state.tx)
                val rotationPower = aimController.calculate(state.tx, 0.0)
                drivetrain.swerveDrive(forwardPower, 0.0, rotationPower, fieldRelative = false)
            } else {
                drivetrain.stop()
            }
        }
    }

    private fun setPivotAngle(target: Angle) = buildCommand("SetAngleCommand") {
        require(pivot)
        runOnce{ pivot.setAngle(target) }
        loopUntil({ pivot.atTarget }){ pivot.setAngle(target) }
        onEnd{ pivot.setIdle() }
    }

    private fun passSerializedNote() = buildCommand("Pass Serialized Note") {
        require(groundIntake, shooter)

        realRobotOnly {
            loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){
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

    private fun shootInAmp() = buildCommand {
        require(noteObserver, shooter, pivot)

        +setPivotAngle(PivotAngle.AMP)

        realRobotOnly {
            loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){
                shooter.outtakeAtAmpSpeed()
            }

            loopUntil({noteObserver.state == NoteObserver.State.NoNote}){
                shooter.outtakeAtAmpSpeed()
            }
        }

        loopForDuration(0.3){
            shooter.outtakeAtAmpSpeed()
        }
    }.withTimeout(4.0)

    private fun shootInSpeaker(
        shooterSpinUpTime: Time = 1.seconds,
        movePivot: Boolean = true
    ) = buildCommand("Shoot In Speaker"){
        require(shooter, groundIntake, pivot)

        fun runShooting(){
            shooter.outtakeAtSpeakerSpeed()
            groundIntake.passToShooterFast()
        }

        val spinupStartTime by getOnceDuringRun{ fpgaTimestamp() }

        runSequenceIf({movePivot}) {
            parallelUntilLeadEnds {
                +setPivotAngle(PivotAngle.SPEAKER)
                loop{ shooter.outtakeAtSpeakerSpeed() }
            }
        }

        loopUntil( { fpgaTimestamp() - spinupStartTime > shooterSpinUpTime } ){
            shooter.outtakeAtSpeakerSpeed()
        }

        realRobotOnly{
            loopUntil({noteObserver.state == NoteObserver.State.NoteInShooter}){ runShooting() }

            loopUntil({noteObserver.state == NoteObserver.State.NoNote}){ runShooting() }
        }

        loopForDuration(0.4){ runShooting() }

        onEnd{
            shooter.setIdle()
            groundIntake.setIdle()
        }
    }.withTimeout(4.0)

    /* Auto Components */

    private fun getAutoCommands() =
        listOf(
            buildCommand("1 Piece Amp + Taxi"){
                +ampAutoStartup()
                +setPivotAngle(PivotAngle.STOWED)
                +followPathOptimal(PathPlannerPath.fromPathFile("AmpSideTaxiShort"))
            },

            buildCommand("2-3 Piece Amp", log = true){
                +ampAutoStartup()
                +driveAndIntake(
                    PathPlannerPath.fromPathFile("AmpGrabG1"),
                    intakeSpinupTime = 0.5.seconds,
                    noteIntakeTime = 0.5.seconds
                )
                +driveAndScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG1"))
                +driveAndIntake(
                    PathPlannerPath.fromPathFile("AmpGrabG2"),
                    noteIntakeTime = 0.5.seconds
                )
                +driveAndScoreAmp(PathPlannerPath.fromPathFile("AmpScoreG2"))
            },

            speakerAutoStartup(AutoStartingPose::getSpeakerLeft).withName("1 Piece Speaker"),

            buildCommand("1 Piece Speaker + Taxi"){
                +speakerAutoStartup(AutoStartingPose::getSpeakerLeft)

                loopForDuration(5){
                    drivetrain.swerveDrive(0.2,0.0,0.0)
                }

                onEnd{ drivetrain.stop() }
            },

            buildCommand("4-5 Piece Speaker"){
                +speakerAutoStartup(AutoStartingPose::getSpeakerCenter)

                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.1"), spinUpShooterDuringPath = true)
                +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.2"))

                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.5"), spinUpShooterDuringPath = true)
                +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.6"))

                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.7"), spinUpShooterDuringPath = true)
                +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.8"))

                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.3"))
                +driveAndScoreSpeaker(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.4"))
            }
        )

    private fun ampAutoStartup() = buildCommand {
        require(drivetrain, pivot, shooter, groundIntake)

        parallelUntilAllEnd {
            runSequence {
                runOnce { drivetrain.resetPose(AutoStartingPose.getAmp()) }
                wait(0.3)
                +followPathOptimal(PathPlannerPath.fromPathFile("DriveToAmp"))
            }

            +setPivotAngle(PivotAngle.AMP)
        }

        +shootInAmp()
    }

    private fun speakerAutoStartup(getStartingPose: () -> Pose2d) = buildCommand {
        require(drivetrain, pivot, shooter, groundIntake)

        runOnce{ drivetrain.resetPose(getStartingPose()) }

        +shootInSpeaker(shooterSpinUpTime = 1.5.seconds)
    }

    private fun driveAndIntake(
        path: PathPlannerPath,
        useGroundIntakeSensor: Boolean = true,
        spinUpShooterDuringPath: Boolean = false,
        intakeSpinupTime: Time = 0.seconds,
        noteIntakeTime: Time = 0.seconds,
    ) = buildCommand("Drive and Intake", log = true){
        require(drivetrain, pivot, shooter, groundIntake)

        fun noteInSerializer(): Boolean {
            return useGroundIntakeSensor &&
                    noteObserver.hasGroundIntakeSensor &&
                    noteObserver.state != NoteObserver.State.NoteInSerializer
        }

        fun getCrosshairOffset(): Double? {
            val currentState = noteObserver.state
            return if (currentState is NoteObserver.State.NoteDetected) currentState.tx else null
        }

        loopForDuration(intakeSpinupTime.inUnit(seconds)){
            groundIntake.intake()
        }

        parallelUntilLeadEnds {
            runSequenceUntil(::noteInSerializer) {
                +(followPathOptimal(path)
                    .onlyWhile{ noteObserver.hasCamera && !noteObserver.noteFound })

                +(noteIntakeDriverAssist{ ChassisPowers(0.3, 0.0, 0.0) }
                    .onlyWhile{ noteObserver.noteFound })

                wait(noteIntakeTime.inUnit(seconds))
            }

            +setPivotAngle(PivotAngle.GROUND_INTAKE_HANDOFF)

            loop{
                groundIntake.intake()
                if (spinUpShooterDuringPath){
                    shooter.outtakeAtSpeakerSpeed()
                }else{
                    shooter.setIdle()
                }
            }
        }
    }

    private fun driveAndScoreAmp(path: PathPlannerPath) = buildCommand("Drive And Score Amp", log = true) {
        require(drivetrain, pivot, shooter, groundIntake)

        parallelUntilAllEnd {
            +followPathOptimal(path)

            runSequence {
                +setPivotAngle(PivotAngle.GROUND_INTAKE_HANDOFF)
                +passSerializedNote()
            }
        }

        +shootInAmp()
    }

    private fun driveAndScoreSpeaker(
        path: PathPlannerPath,
        shooterSpinUpTime: Time = 0.seconds
    ) = buildCommand {
        require(drivetrain, pivot, shooter, groundIntake)

        parallelUntilLeadEnds {
            +followPathOptimal(path)

            loop{
                shooter.outtakeAtSpeakerSpeed()
                groundIntake.setIdle()
                pivot.setAngle(PivotAngle.SPEAKER)
            }
        }

        +shootInSpeaker(shooterSpinUpTime)
    }
}