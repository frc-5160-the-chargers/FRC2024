package frc.robot.rigatoni

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.controls.PIDController
import kcommand.commandbuilder.buildCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.Tunable
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.wpilibextensions.Cmd
import frc.chargers.utils.squareMagnitude
import frc.chargers.wpilibextensions.distanceTo
import frc.chargers.wpilibextensions.flipWhenRedAlliance
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.utils.onDoubleClick
import frc.robot.rigatoni.subsystems.*
import frc.robot.rigatoni.subsystems.Shooter
import monologue.Monologue
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
    
    private val driverController = DriverController(DRIVER_CONTROLLER_PORT)
    private val driverTouchpad = driverController.touchpad()
    private val operatorController = CommandXboxController(OPERATOR_CONTROLLER_PORT)

    private val autoChooser = SendableChooser<Command>()

    init {
        // ---------- Generic Setup ----------
        DriverStation.silenceJoystickConnectionWarning(true)
        //PortForwarder.add(5800, "photonvision.local", 5800)
        gyro.simHeadingSource = { drivetrain.calculatedHeading }
        Tunable.tuningMode = true

        // ---------- Logging Config ----------
        Monologue.setupMonologue(
            this,
            "",
            Monologue.MonologueConfig()
                .withFileOnly { DriverStation.isFMSAttached() }
        )
        runPeriodic { Monologue.updateAll() }
        try {
            SmartDashboard.putData("PDHData", PowerDistribution(1, PowerDistribution.ModuleType.kRev))
        }catch (_: Exception){
            Alert("PDH Not Found", AlertType.kError).set(true)
        }

        // -------- Generic Config -----------
        setDefaultCommands()
        setButtonBindings()
        setEventListeners()

        // ---------- Auto Config ----------
        autoChooser.setDefaultOption(
            "Taxi",
            Cmd.run(drivetrain){ drivetrain.swerveDrive(0.2, 0.0, 0.0, fieldRelative = true) }
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

    private fun setEventListeners() {
        val driverControllerAlert = Alert("Driver Controller not connected!", AlertType.kWarning)
        val operatorControllerAlert = Alert("Operator Controller not connected!", AlertType.kWarning)
        Trigger{ !DriverStation.isJoystickConnected(DRIVER_CONTROLLER_PORT) }
            .whileTrue(Cmd.run{ driverControllerAlert.set(true) }.ignoringDisable(true))
            .onFalse(Cmd.runOnce { driverControllerAlert.set(false) })

        Trigger { !DriverStation.isJoystickConnected(OPERATOR_CONTROLLER_PORT) }
            .whileTrue(Cmd.run{ operatorControllerAlert.set(true) }.ignoringDisable(true))
            .onFalse(Cmd.runOnce { operatorControllerAlert.set(false) })
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

            touchpad().onDoubleClick(Cmd.runOnce { gyro.zeroHeading(180.degrees) })

            var current = 0.degrees
            for (trigger in aimTriggers){
                trigger.whileTrue(AngleAimCommand(current, drivetrain) { driverController.swerveOutput })
                current -= 90.degrees
            }

            noteIntakeTrigger.whileTrue(
                Cmd.run(groundIntake) { groundIntake.intake() }
            )

            noteIntakeTrigger.and { noteObserver.shouldStartPursuit }
                .whileTrue(noteIntakeDriverAssist())

            noteIntakeTrigger.whileTrue(
                noteIntakeDriverAssist()
                    .alongWith(Cmd.run(groundIntake){ groundIntake.intake() })
            )

            climbUpTrigger.whileTrue(
                Cmd.run(climber){
                    climber.moveLeftHook(1.0)
                    climber.moveRightHook(1.0)
                }
            )

            climbDownTrigger.whileTrue(
                Cmd.run(climber){
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
                Cmd.run(groundIntake){ groundIntake.intake() }
            )
            leftTrigger().whileTrue(
                Cmd.run(groundIntake, shooter){ // ground outtake command
                    groundIntake.outtake()
                    shooter.setVoltage((-6).volts)
                }
            )

            rightBumper().and(leftBumper().negate()).whileTrue(passNoteToShooter())
            leftBumper().and(rightBumper().negate()).whileTrue(
                Cmd.run(shooter, pivot){ // spinup shooter command
                    shooter.outtakeAtSpeakerSpeed()
                    pivot.setAngle(PivotAngle.SPEAKER)
                }
            )
            leftBumper().and(rightBumper()).whileTrue(
                shootInSpeaker(shooterSpinUpTime = 0.3.seconds)
            )

            start().onDoubleClick(
                Cmd.runOnce { drivetrain.reSyncRelativeEncoders() }
            )
        }
    }

    private fun setDefaultCommands(){
        drivetrain.defaultCommand = Cmd.run(drivetrain) {
            drivetrain.swerveDrive(driverController.swerveOutput, fieldRelative = !driverTouchpad.asBoolean)
        }

        shooter.defaultCommand = Cmd.run(shooter) {
            var speed = operatorController.leftY// applyDeadband(operatorController.leftY, SHOOTER_DEADBAND)
            speed *= SHOOTER_SPEED_MULTIPLIER
            log("OperatorController/ShooterSpeed", speed)
            shooter.setSpeed(speed)
        }

        val pivotSpeedInvert = if (RobotBase.isReal()) -1.0 else 1.0
        pivot.defaultCommand = Cmd.run(pivot) {
            var speed = applyDeadband(operatorController.rightY, PIVOT_DEADBAND)
            speed = (speed * pivotSpeedInvert).squareMagnitude()
            speed *= PIVOT_SPEED_MULTIPLIER

            pivot.setSpeed(speed)
            log("OperatorController/PivotSpeed", speed)
        }.finallyDo(pivot::setIdle)

        climber.defaultCommand = Cmd.run(climber) { climber.setIdle() }
        groundIntake.defaultCommand = Cmd.run(groundIntake) { groundIntake.setIdle() }
    }

    private fun shouldPathFind(path: PathPlannerPath): Boolean {
        val pathStartPose = path.previewStartingHolonomicPose.flipWhenRedAlliance()
        val pathEndPose = path.pathPoses.last().flipWhenRedAlliance()
        val drivetrainPose = drivetrain.robotPose

        return drivetrainPose.distanceTo(pathStartPose) > ACCEPTABLE_DISTANCE_BEFORE_PATHFIND &&
                drivetrainPose.distanceTo(pathEndPose) > drivetrainPose.distanceTo(pathStartPose)
    }

    /* Commands */

    private fun noteIntakeDriverAssist(
        requestedSpeedSupplier: () -> ChassisPowers = { driverController.swerveOutput }
    ): Command {
        fun getNotePursuitSpeed(txValue: Double): Double {
            val swerveOutput = requestedSpeedSupplier()
            return max(abs(swerveOutput.xPower), abs(swerveOutput.yPower)) * (1.0 - txValue / 50.0) // scales based off of the vision target error
        }

        val aimController = PIDController(AIM_TO_NOTE_PID)

        return Cmd.run(drivetrain) {
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
        }.withName("NoteIntakeDriverAssist")
    }

    private fun pivotAngleCommand(target: Angle) =
        Cmd.sequence(
            Cmd.runOnce(pivot) { pivot.setAngle(target) },
            Cmd.run { pivot.setAngle(target) }.until { pivot.atTarget },
            Cmd.runOnce { pivot.setIdle() }
        ).withName("PivotAngleCommand")

    private fun waitForNoteToPass(additionalDelaySecs: Double) =
        Cmd.sequence(
            Cmd.waitUntil { isSimulation() || noteObserver.state == NoteState.InShooter },
            Cmd.waitUntil { isSimulation() || noteObserver.state == NoteState.None },
            Cmd.waitSeconds(additionalDelaySecs)
        ).withName("WaitForNoteToPass")

    private fun passNoteToShooter() =
        Cmd.sequence(
            Cmd.runOnce(shooter, groundIntake) {
                // motors will continue running until we stop them
                shooter.receiveFromGroundIntake()
                groundIntake.passToShooterSlow()
            },
            Cmd.waitUntil { isSimulation() || noteObserver.state == NoteState.InShooter },
            Cmd.waitSeconds(0.4),
            Cmd.runOnce {
                shooter.setIdle()
                groundIntake.setIdle()
            }
        ).withName("PassNoteToShooter")

    private fun shootInAmp() =
        Cmd.sequence(
            pivotAngleCommand(PivotAngle.AMP),
            Cmd.runOnce(shooter) { shooter.outtakeAtAmpSpeed() },
            waitForNoteToPass(0.25),
            Cmd.runOnce { shooter.setIdle() }
        ).withTimeout(4.0).withName("ShootInAmp")

    /**
     * Accelerates a note out of the serializer and through the shooter.
     * Presumes that the shooter is already spinning.
     */
    private fun runNoteThroughShooter() =
        Cmd.sequence(
            Cmd.runOnce(groundIntake) { groundIntake.passToShooterFast() },
            waitForNoteToPass(0.4),
            Cmd.runOnce { groundIntake.setIdle() }
        ).withTimeout(1.5)


    // Note: pivot.setAngle() is the method call,
    // while pivotAngleCommand() is a command that ends by itself.

    /**
     * Runs the full shooting sequence.
     */
    private fun shootInSpeaker(shooterSpinUpTime: Time = 1.seconds, movePivot: Boolean = true) =
        Cmd.sequence(
            Cmd.runOnce(shooter) {
                shooter.outtakeAtSpeakerSpeed()
            },
            Cmd.parallel( // finishes when all are done
                Cmd.wait(shooterSpinUpTime),
                Cmd.waitUntil { !movePivot || pivot.atTarget },
                if (movePivot) pivotAngleCommand(PivotAngle.SPEAKER) else Cmd.none()
            ),
            runNoteThroughShooter(),
            Cmd.runOnce {
                shooter.setIdle()
                pivot.setIdle()
                groundIntake.setIdle()
            }
        )

    /* Auto Components */

    private fun getAutoCommands() =
        listOf(
            Cmd.sequence(
                ampAutoStartup(),
                pivotAngleCommand(PivotAngle.STOWED),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxiShort"))
            ).withName("1 Piece Amp"),

            buildCommand("2-3 Piece Amp", log = true){
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

            buildCommand("2 piece amp non-close", log = true){
                +ampAutoStartup()
                +pivotAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)
                +driveAndIntake(
                    PathPlannerPath.fromChoreoTrajectory("AmpAutoFar.1"),
                    noteIntakeTime = 0.5.seconds
                )
                +driveThenScoreAmp(PathPlannerPath.fromChoreoTrajectory("AmpAutoFar.2"))
                +pivotAngleCommand(PivotAngle.STOWED)
            },

            speakerAutoStartup(AutoStartingPose::getSpeakerLeft).withName("1 Piece Speaker"),

            buildCommand("1 Piece Speaker + Taxi", log = true){
                +speakerAutoStartup(AutoStartingPose::getSpeakerLeft)

                loopForDuration(5){
                    drivetrain.swerveDrive(0.2,0.0,0.0)
                }

                onEnd{ drivetrain.stop() }
            },

            buildCommand("4.5 Piece Speaker", log = true){
                +speakerAutoStartup(AutoStartingPose::getSpeakerCenter)
                repeat(3) { i ->
                    +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 1}"))
                    +driveThenShoot(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 2}"))
                }
                +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.7"))
            },

            buildCommand("4.5 Piece Speaker(Continuous Shooter Running)", log = true) {
                +speakerAutoStartup(AutoStartingPose::getSpeakerCenter)
                parallel {
                    runSequence {
                        repeat(3) { i ->
                            +driveAndIntake(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 1}"))
                            +AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("5pAutoCenter.${2 * i + 2}"))
                            +runNoteThroughShooter()
                        }
                    }.asDeadline()

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
                +(AutoBuilder.followPath(path).until { noteObserver.shouldStartPursuit })

                +(noteIntakeDriverAssist { ChassisPowers(0.3, 0.0, 0.0) }
                    .withTimeout(3.0)
                    .onlyWhile { noteObserver.hasCamera && !noteObserver.noteInRobot })

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

    private fun driveThenScoreAmp(path: PathPlannerPath) =
        Cmd.parallel(
            Cmd.sequence(
                pivotAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF),
                passNoteToShooter()
            ),
            AutoBuilder.followPath(path)
        ).andThen(shootInAmp()).withName("DriveThenScore(Amp)")

    private fun driveThenShoot(path: PathPlannerPath, shooterSpinUpTime: Time = 0.seconds) =
        Cmd.sequence(
            Cmd.parallel(
                Cmd.run(shooter, groundIntake, pivot) {
                    shooter.outtakeAtSpeakerSpeed()
                    groundIntake.setIdle()
                    pivot.setAngle(PivotAngle.SPEAKER)
                },
                deadlineCommand = AutoBuilder.followPath(path)
            ),
            shootInSpeaker(shooterSpinUpTime)
        ).withName("DriveThenShoot(Speaker)")
}