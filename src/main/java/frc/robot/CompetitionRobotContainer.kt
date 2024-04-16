// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.Orchestra
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.commands.loopCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.constants.DashboardTuner
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.MotorData
import frc.chargers.hardware.motorcontrol.rev.util.PeriodicFrameConfig
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.absolute.ChargerDutyCycleEncoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.subsystems.swervedrive.*
import frc.chargers.utils.registerSingletonsForAutoLogOutput
import frc.robot.commands.*
import frc.robot.commands.aiming.pursueNoteElseTeleopDrive
import frc.robot.commands.auto.AutoChooser
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorInterface
import frc.robot.hardware.subsystems.climber.Climber
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIOReal
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIOSim
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIOReal
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIOSim
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.pivot.PivotEncoderType
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIOReal
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIOSim
import frc.robot.hardware.subsystems.shooter.NoteVisualizer
import frc.robot.hardware.subsystems.shooter.Shooter
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOReal
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOSim
import frc.robot.hardware.subsystems.vision.VisionManager
import kotlin.jvm.optionals.getOrNull


/**
 * The robot container for the official competition robot.
 *
 * IMPORTANT: Ground intake side is considered the front of the robot.
 * Pivot side is considered the back.
 */
class CompetitionRobotContainer: ChargerRobotContainer() {

    private val shooterRatio = 1.698
    private val pivotRatio = 96.0
    private val groundIntakeRatio = 15.0 / 12.0
    private val conveyorRatio = 7.5 / 1.0

    private val rickRoller = Orchestra()

    // note of reference: an IO class is a low-level component of the robot
    // that integrates advantagekit logging.'
    private val gyroIO = ChargerNavX(useFusedHeading = false).apply{
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red){
            zeroHeading(180.degrees)
        }else{
            zeroHeading(0.degrees)
        }
    }

    private val shooter = Shooter(
        if (isReal()){
            ShooterIOReal(
                beamBreakSensor = DigitalInput(SHOOTER_SENSOR_ID),
                topMotor = ChargerSparkFlex(SHOOTER_MOTOR_ID){
                    periodicFrameConfig = PeriodicFrameConfig.Optimized(
                        utilizedData = listOf(MotorData.VELOCITY, MotorData.VOLTAGE, MotorData.TEMPERATURE)
                    )
                    inverted = true
                    smartCurrentLimit = SmartCurrentLimit(60.amps)
                },
                gearRatio = shooterRatio
            )
        }else{
            ShooterIOSim(
                DCMotorSim(DCMotor.getNeoVortex(1), shooterRatio, 0.004)
            )
        },
        shootingFFEquation = AngularMotorFFEquation(0.0, 0.0),
        shootingPID = PIDConstants(0.2,0,0),
        closedLoopSpeakerShooting = false // tbd for now
    )


    private val pivot = Pivot(
        if (isReal()){
            PivotIOReal(
                ChargerSparkMax(PIVOT_MOTOR_ID){
                    periodicFrameConfig = PeriodicFrameConfig.Optimized(
                        utilizedData = listOf(MotorData.POSITION)
                    )
                    smartCurrentLimit = SmartCurrentLimit(35.amps)
                },
                useOnboardPID = false,
                encoderType = PivotEncoderType.ExternalAbsoluteEncoder(
                    absoluteEncoder = ChargerDutyCycleEncoder(PIVOT_ENCODER_ID){ inverted = true },
                    motorGearRatio = pivotRatio,
                    offset = (-0.23).rotations
                ),
            )
        }else{
            PivotIOSim(DCMotorSim(DCMotor.getNEO(1), pivotRatio, 0.004))
        },
        ///// FOR NAYAN: Pivot PID Constants
        if (isReal()) PIDConstants(7.0,0,0) else PIDConstants(10.0, 0.0, 0.0),
        AngularTrapezoidProfile(
            maxVelocity = AngularVelocity(8.0),
            maxAcceleration = AngularAcceleration(10.0)
        ),
        forwardSoftStop = 1.636.radians,
        reverseSoftStop = (-1.8).radians
    )

    private val groundIntake = GroundIntakeSerializer(
        if (isReal()){
            GroundIntakeIOReal(
                topMotor = ChargerTalonFX(GROUND_INTAKE_ID).also{ rickRoller.addInstrument(it) },
                conveyorMotor = ChargerSparkMax(CONVEYOR_ID){
                    periodicFrameConfig = PeriodicFrameConfig.Optimized(
                        utilizedData = listOf(MotorData.VELOCITY, MotorData.VOLTAGE, MotorData.TEMPERATURE)
                    )
                    inverted = true
                    smartCurrentLimit = SmartCurrentLimit(45.amps)
                },
                intakeGearRatio = groundIntakeRatio,
                //beamBreakSensor = DigitalInput(GROUND_INTAKE_SENSOR_ID)
            )
        }else{
            GroundIntakeIOSim(
                topMotorSim = DCMotorSim(DCMotor.getNeoVortex(1), groundIntakeRatio, 0.10),
                conveyorMotorSim = DCMotorSim(DCMotor.getNEO(1), conveyorRatio, 0.004)
            )
        }
    )


    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            ChargerSparkMax(DrivetrainID.TL_TURN),
            ChargerSparkMax(DrivetrainID.TR_TURN){ inverted = true },
            ChargerSparkMax(DrivetrainID.BL_TURN),
            ChargerSparkMax(DrivetrainID.BR_TURN)
        ){
            periodicFrameConfig = PeriodicFrameConfig.Optimized(
                utilizedData = listOf(MotorData.VOLTAGE, MotorData.TEMPERATURE)
            )
            smartCurrentLimit = SmartCurrentLimit(30.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        },
        turnEncoders = swerveCANcoders(
            topLeft = ChargerCANcoder(DrivetrainID.TL_ENCODER),
            topRight = ChargerCANcoder(DrivetrainID.TR_ENCODER),
            bottomLeft = ChargerCANcoder(DrivetrainID.BL_ENCODER),
            bottomRight = ChargerCANcoder(DrivetrainID.BR_ENCODER),
            useAbsoluteSensor = true
        ).withOffsets(
            topLeftZero = 0.621.radians,
            topRightZero = 1.37.radians,
            bottomLeftZero = 4.971.radians,
            bottomRightZero = 6.243.radians,
        ),
        driveMotors = talonFXSwerveMotors(
            ChargerTalonFX(DrivetrainID.TL_DRIVE),
            ChargerTalonFX(DrivetrainID.TR_DRIVE){ inverted = true },
            ChargerTalonFX(DrivetrainID.BL_DRIVE),
            ChargerTalonFX(DrivetrainID.BR_DRIVE){ inverted = true },
        ){
            statorCurrentLimitEnable = true
            statorCurrentLimit = 120.amps
            supplyCurrentLimitEnable = true
            supplyCurrentLimit = 60.amps
            neutralMode = NeutralModeValue.Brake
        }.also{ it.forEach{ motor -> rickRoller.addInstrument(motor) } },
        controlData = SwerveControlData(
            azimuthControl = SwerveAzimuthControl.PID(
                PIDConstants(7.0,0,0.1),
            ),
            openLoopDiscretizationRate = 2.0,
            closedLoopDiscretizationRate = 0.0,
            velocityPID = PIDConstants(0.02,0.0,0.0),
            velocityFF = if (isReal()){
                AngularMotorFFEquation(0.11, 0.17)
            }else{
                AngularMotorFFEquation(0.0, 0.128)
            },
            robotRotationPID = PIDConstants(4.5, 0, 0.001),
            robotTranslationPID = PIDConstants(13.0,0,0.001)
        ),
        useOnboardPID = false,
        hardwareData = SwerveHardwareData.mk4iL2(
            turnMotorType = DCMotor.getNEO(1),
            driveMotorType = DCMotor.getKrakenX60(1),
            maxModuleSpeed = 4.5.meters / 1.seconds,
            trackWidth = 32.inches, wheelBase = 32.inches,
        ),
        gyro = if (isReal()) gyroIO else null,
    )

    private val climber = Climber(
        if (isReal()){
            ClimberIOReal(
                leftMotor = ChargerSparkMax(CLIMBER_ID_LEFT){
                    periodicFrameConfig = PeriodicFrameConfig.Optimized(
                        utilizedData = listOf(MotorData.POSITION, MotorData.VOLTAGE)
                    )
                    inverted = false
                    idleMode = CANSparkBase.IdleMode.kBrake
                },
                rightMotor = ChargerSparkMax(CLIMBER_ID_RIGHT){
                    periodicFrameConfig = PeriodicFrameConfig.Optimized(
                        utilizedData = listOf(MotorData.POSITION, MotorData.VOLTAGE)
                    )
                    inverted = true
                    idleMode = CANSparkBase.IdleMode.kBrake
                }
            )
        }else{
            ClimberIOSim(
                DCMotorSim(DCMotor.getNEO(1), 10.0, 0.02),
                DCMotorSim(DCMotor.getNEO(1), 10.0, 0.02),
            )
        },
        highLimit = -100.radians,
    )

    private val vision = VisionManager(drivetrain.poseEstimator, tunableCamerasInSim = false)

    init{
        // disabled due to potential memory issues
        /*
        if (isReal() || hasReplaySource()){
            drivetrain.poseEstimator = ThreadedPoseMonitor(
                kinematics = drivetrain.kinematics,
                hardwareData = drivetrain.hardwareData,
                navX = gyroIO,
                turnMotors = TURN_MOTORS,
                absoluteEncoders = TURN_ENCODERS,
                driveMotors = DRIVE_MOTORS
            )
        }
         */

        registerSingletonsForAutoLogOutput(DriverController, OperatorInterface)

        DriverStation.silenceJoystickConnectionWarning(true)
        DashboardTuner.tuningMode = false
        LiveWindow.disableAllTelemetry()

        configureBindings()
        configureDefaultCommands()

        if (isSimulation()){
            // vision pose estimation not tested on real robot
            vision.enableVisionPoseEstimation()

            IMUSimulation.configure(
                headingSupplier = { drivetrain.heading },
                chassisSpeedsSupplier = { drivetrain.currentSpeeds }
            )

            NoteVisualizer.setRobotPoseSupplier { drivetrain.poseEstimator.robotPose }
            NoteVisualizer.setLauncherTransformSupplier { pivot.mechanism3dPose }
        }
    }

    private fun configureDefaultCommands(){
        // setDefaultRunCommand gives context blocks which allow for an implicit "this" for the subsystem being called
        // for instance, you can do drive.setDefaultRunCommand{ drive() }
        // instead of drive.setDefaultRunCommand{ drivetrain.drive() }
        drivetrain.setDefaultRunCommand{
            swerveDrive(
                DriverController.swerveOutput,
                fieldRelative = !DriverController.shouldDisableFieldRelative
            )
        }

        shooter.setDefaultRunCommand{
            val speed = OperatorInterface.shooterSpeedAxis()
            if (speed > 0.0){
                outtake(speed)
            }else{
                intake(speed)
            }
        }

        pivot.setDefaultRunCommand(endBehavior = pivot::setIdle){
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

        groundIntake.setDefaultRunCommand{
            setIdle()
        }
    }

    private fun configureBindings(){
        fun resetAimToAngle() = runOnceCommand{
            drivetrain.removeRotationOverride()
        }

        fun targetAngle(heading: Angle) = runOnceCommand{
            // used to make pivot side the front instead of the ground intake side
            val targetAngleOffset = 180.degrees

            val allianceAngleCompensation = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                180.degrees
            } else {
                0.degrees
            }

            drivetrain.setRotationOverride(
                AimToAngleRotationOverride(
                    heading + allianceAngleCompensation + targetAngleOffset,
                    ANGLE_TO_ROTATIONAL_VELOCITY_PID
                )
            )
        }

        DriverController.apply{
            pointNorthTrigger.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())

            pointEastTrigger.onTrue(targetAngle(-90.degrees)).onFalse(resetAimToAngle())

            pointSouthTrigger.onTrue(targetAngle(-180.degrees)).onFalse(resetAimToAngle())

            pointWestTrigger.onTrue(targetAngle(-270.degrees)).onFalse(resetAimToAngle())

            zeroHeadingTrigger.onTrue(runOnceCommand{ gyroIO.zeroHeading(180.degrees) })

            driveToNoteAssistTrigger.whileTrue(pursueNoteElseTeleopDrive(drivetrain, vision.notePipeline))
        }

        // loopCommand and runOnceCommand are wrappers around RunCommand and InstantCommand
        // which allows for outer lambda block syntax
        OperatorInterface.apply{
            groundIntakeTrigger.whileTrue(runGroundIntake(groundIntake, shooter))

            groundOuttakeTrigger.whileTrue(
                loopCommand(groundIntake, shooter){
                    groundIntake.outtake()
                    shooter.outtake(-6.volts)
                }
            )

            passToShooterTrigger.whileTrue(passSerializedNote(groundIntake, shooter))

            spinUpShooterTrigger.whileTrue(
                loopCommand(shooter, pivot){
                    shooter.outtakeAtSpeakerSpeed()
                    pivot.setAngle(PivotAngle.SPEAKER)
                }
            )

            shootInSpeakerTrigger.whileTrue(
                shootInSpeaker(shooter, groundIntake, pivot, shooterSpinUpTime = 0.3.seconds)
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
                loopCommand(shooter){
                    shooter.outtakeAtSpeakerSpeed()
                }
            )
        }

        Trigger{ groundIntake.hasNote && groundIntake.hasNoteDetector }
            .onTrue(runOnceCommand{ DriverController.rumbleProcessor.setRumble(GenericHID.RumbleType.kBothRumble, 1.0); println("Rumble called!") })
            .onFalse(WaitCommand(0.1).andThen(runOnceCommand{ DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) }))
    }




    private val autoChooser = AutoChooser(vision.notePipeline, drivetrain, shooter, pivot, groundIntake)

    private val testCommandChooser = TestCommandChooser(drivetrain, shooter, vision)

    override val testCommand: Command
        get() = testCommandChooser.selected

    override val autonomousCommand: Command
        get() = if (isSimulation()) autoChooser.ampAutoTest else autoChooser.selected
}
