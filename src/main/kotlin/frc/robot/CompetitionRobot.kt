package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import com.revrobotics.CANSparkBase
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.RunCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.MotorData
import frc.chargers.hardware.motorcontrol.rev.util.PeriodicFrameConfig
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.ChargerDutyCycleEncoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.sensors.withOffset
import frc.chargers.hardware.subsystems.swervedrive.*
import frc.chargers.utils.Precision
import frc.robot.commands.noteIntakeDriverAssist
import frc.robot.commands.passSerializedNote
import frc.robot.commands.shootInSpeaker
import frc.robot.inputdevices.DriverController
import frc.robot.inputdevices.OperatorInterface
import frc.robot.subsystems.Climber
import frc.robot.subsystems.GroundIntakeSerializer
import frc.robot.subsystems.NoteObserver
import frc.robot.subsystems.pivot.Pivot
import frc.robot.subsystems.pivot.PivotAngle
import frc.robot.subsystems.pivot.PivotEncoderType
import frc.robot.subsystems.shooter.Shooter
import org.photonvision.PhotonCamera
import kotlin.jvm.optionals.getOrNull

/**
 * Our competition robot; rigatoni.
 */
@Suppress("unused")
class CompetitionRobot: ChargerRobot(){
    val gyro = ChargerNavX()

    val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = if (isSimulation()){
            SwerveData.create{ MotorSim(DCMotor.getNEO(1), moi = 0.0000087.ofUnit(kilo.grams * (meters * meters))) }
        }else{
            SwerveData(
                topLeft = ChargerSparkMax(DrivetrainID.TL_TURN),
                topRight = ChargerSparkMax(DrivetrainID.TR_TURN){ inverted = true },
                bottomLeft = ChargerSparkMax(DrivetrainID.BL_TURN),
                bottomRight = ChargerSparkMax(DrivetrainID.BR_TURN)
            ){ // configure lambda; configuration below are applied to all motors
                periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.VELOCITY, MotorData.VOLTAGE, MotorData.CURRENT))
                smartCurrentLimit = SmartCurrentLimit(30.amps)
                voltageCompensationNominalVoltage = 12.volts
                openLoopRampRate = 48.0
                closedLoopRampRate = 48.0
            }
        },
        turnEncoders = if (isSimulation()){
            SwerveData.create{ null }
        }else{
            SwerveData(
                topLeft = ChargerCANcoder(DrivetrainID.TL_ENCODER).absolute.withOffset(0.621.radians),
                topRight = ChargerCANcoder(DrivetrainID.TR_ENCODER).absolute.withOffset(1.37.radians),
                bottomLeft = ChargerCANcoder(DrivetrainID.BL_ENCODER).absolute.withOffset(4.971.radians),
                bottomRight = ChargerCANcoder(DrivetrainID.BR_ENCODER).absolute.withOffset(6.243.radians)
            )
        },
        driveMotors = if (isSimulation()) {
            SwerveData.create{ MotorSim(DCMotor.getKrakenX60(1), moi = 0.000054.ofUnit(kilo.grams * (meters * meters))) }
        }else{
            SwerveData(
                topLeft = ChargerTalonFX(DrivetrainID.TL_DRIVE),
                topRight = ChargerTalonFX(DrivetrainID.TR_DRIVE){ inverted = true },
                bottomLeft = ChargerTalonFX(DrivetrainID.BL_DRIVE),
                bottomRight = ChargerTalonFX(DrivetrainID.BR_DRIVE){ inverted = true },
            ){ // configure lambda; configuration below are applied to all motors
                statorCurrentLimitEnable = true
                statorCurrentLimit = 120.amps
                supplyCurrentLimitEnable = true
                supplyCurrentLimit = 60.amps
                neutralMode = NeutralModeValue.Brake
            }
        },
        chassisConstants = SwerveChassisConstants(
            trackWidth = 27.inches,
            wheelBase = 27.inches,
        ),
        moduleConstants = SwerveModuleConstants.mk4iL2(
            useOnboardPID = false,
            turnMotorControlScheme = SwerveAzimuthControl.PID(PIDConstants(7.0,0,0), Precision.Within(1.degrees)),
            velocityPID = PIDConstants(0.05,0,0),
            velocityFF = AngularMotorFFEquation(0.0,0.13),
        ),
        gyro = if (isSimulation()) null else gyro
    )

    val pivot = Pivot(
        if (isSimulation()){
            MotorSim(DCMotor.getNEO(1))
        }else{
            ChargerSparkMax(PIVOT_MOTOR_ID){
                periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.POSITION))
                smartCurrentLimit = SmartCurrentLimit(35.amps)
            }
        },
        PivotEncoderType.ExternalAbsoluteEncoder(
            ChargerDutyCycleEncoder(PIVOT_ENCODER_ID),
            motorGearRatio = 96.0,
            offset = (-0.23).rotations
        ),
        useOnboardPID = false,
        PIDConstants(7.0,0,0.001),
        AngularTrapezoidProfile(
            maxVelocity = AngularVelocity(8.0),
            maxAcceleration = AngularAcceleration(10.0)
        ),
        forwardSoftStop = 1.636.radians,
        reverseSoftStop = (-1.8).radians
    )

    val groundIntake = GroundIntakeSerializer(
        groundIntakeMotor = if (isSimulation()){
            MotorSim(DCMotor.getFalcon500(1))
        }else{
            ChargerTalonFX(GROUND_INTAKE_ID)
        },
        serializerMotor = if (isSimulation()){
            MotorSim(DCMotor.getNEO(1))
        }else{
            ChargerSparkMax(CONVEYOR_ID){
                periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.VELOCITY, MotorData.VOLTAGE))
                inverted = true
                smartCurrentLimit = SmartCurrentLimit(45.amps)
            }
        },
        groundIntakeGearRatio = 15.0 / 12.0,
        serializerGearRatio = 7.5 / 1.0
    )

    val shooter = Shooter(
        if (isSimulation()){
            MotorSim(DCMotor.getNeoVortex(1))
        }else{
            ChargerSparkFlex(SHOOTER_MOTOR_ID){
                periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.VELOCITY, MotorData.VOLTAGE))
                inverted = true
                smartCurrentLimit = SmartCurrentLimit(60.amps)
            }
        },
        gearRatio = 1.698
    )

    val climber = Climber(
        leftMotor = ChargerSparkMax(CLIMBER_ID_LEFT){
            periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.POSITION, MotorData.VOLTAGE))
            inverted = false
            idleMode = CANSparkBase.IdleMode.kBrake
        },
        rightMotor = ChargerSparkMax(CLIMBER_ID_RIGHT){
            periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.POSITION, MotorData.VOLTAGE))
            inverted = true
            idleMode = CANSparkBase.IdleMode.kBrake
        },
        highLimit = (-100).radians,
        gearRatio = 10.0
    )

    val noteObserver = NoteObserver(
        groundIntakeSensor = DigitalInput(GROUND_INTAKE_SENSOR_ID),
        shooterSensor = DigitalInput(SHOOTER_SENSOR_ID),
        noteDetectorCamera = PhotonCamera("MLWebcam"),
        cameraHeight = 10.inches,
        cameraPitch = 37.degrees
    )

    override fun robotInit(){
        DriverController

        DriverStation.silenceJoystickConnectionWarning(true)

        IMUSimulation.configure(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )

        SmartDashboard.putData(
            "Power Distribution",
            PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        )

        setDefaultCommands()
        setButtonBindings()

        autonomous().whileTrue(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("RandomStuff"))
        )
    }

    private fun setDefaultCommands(){
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(DriverController.swerveOutput)
        }

        shooter.setDefaultRunCommand{
            val speed = OperatorInterface.shooterSpeedAxis()
            if (speed > 0.0 && noteObserver.noteInRobot){
                setIdle()
            }else{
                setSpeed(speed)
            }
        }

        pivot.setDefaultRunCommand(endBehavior = { pivot.setIdle() }){
            setSpeed(OperatorInterface.pivotSpeedAxis())
        }

        climber.setDefaultRunCommand{
            if (DriverController.climbersUpTrigger.asBoolean){
                moveLeftHook(1.0)
                moveRightHook(1.0)
            }else if (DriverController.climbersDownTrigger.asBoolean){
                moveLeftHook(-1.0)
                moveRightHook(-1.0)
            }else{
                setIdle()
            }
        }

        groundIntake.setDefaultRunCommand{ setIdle() }
    }

    private fun setButtonBindings() {
        fun resetAimToAngle() = InstantCommand{
            drivetrain.removeRotationOverride()
        }

        fun targetAngle(heading: Angle) = InstantCommand{
            // used to make pivot side the front instead of the ground intake side
            val targetAngleOffset = 180.degrees

            val allianceAngleCompensation = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                180.degrees
            } else {
                0.degrees
            }

            drivetrain.setRotationOverride(
                AimToAngleRotationOverride(
                    { heading + allianceAngleCompensation + targetAngleOffset },
                    ANGLE_TO_ROTATIONAL_VELOCITY_PID
                )
            )
        }

        DriverController.apply{
            pointNorthTrigger.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())

            pointEastTrigger.onTrue(targetAngle(-90.degrees)).onFalse(resetAimToAngle())

            pointSouthTrigger.onTrue(targetAngle(-180.degrees)).onFalse(resetAimToAngle())

            pointWestTrigger.onTrue(targetAngle(-270.degrees)).onFalse(resetAimToAngle())

            zeroHeadingTrigger.onTrue(InstantCommand{ gyro.zeroHeading(180.degrees) })

            driveToNoteAssistTrigger.whileTrue(noteIntakeDriverAssist(drivetrain, noteObserver))
        }

        // loopCommand and runOnceCommand are wrappers around RunCommand and InstantCommand
        // which allows for outer lambda block syntax
        OperatorInterface.apply{
            groundIntakeTrigger.whileTrue(
                RunCommand(groundIntake){ groundIntake.intake() }
            )

            groundOuttakeTrigger.whileTrue(
                RunCommand(groundIntake, shooter){
                    groundIntake.outtake()
                    shooter.setVoltage((-6).volts)
                }
            )

            passToShooterTrigger.whileTrue(passSerializedNote(noteObserver, groundIntake, shooter))

            spinUpShooterTrigger.whileTrue(
                RunCommand(shooter, pivot){
                    shooter.outtakeAtSpeakerSpeed()
                    pivot.setAngle(PivotAngle.SPEAKER)
                }
            )

            shootInSpeakerTrigger.whileTrue(
                shootInSpeaker(noteObserver, shooter, groundIntake, pivot, shooterSpinUpTime = 0.3.seconds)
            )

            // when only held, the buttons will just cause the pivot to PID to the appropriate position
            // interrupt behavior set as to prevent command scheduling conflicts
            ampPositionTrigger.whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.AMP)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            sourcePositionTrigger.whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.SOURCE)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            stowPivotTrigger.whileTrue(pivot.setAngleCommand(PivotAngle.STOWED))

            y().whileTrue(
                RunCommand(shooter){
                    shooter.outtakeAtSpeakerSpeed()
                }
            )
        }
    }
}
