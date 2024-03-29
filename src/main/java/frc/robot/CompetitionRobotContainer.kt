// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularAcceleration
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.path.PathPlannerPath
import com.revrobotics.CANSparkBase
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj2.command.Command
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
import frc.chargers.hardware.inputdevices.onClickAndHold
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.MotorData
import frc.chargers.hardware.motorcontrol.rev.util.PeriodicFrameConfig
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.absolute.ChargerDutyCycleEncoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
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

    // note of reference: an IO class is a low-level component of the robot
    // that integrates advantagekit logging.

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
                beamBreakSensor = DigitalInput(BEAM_BREAK_SENSOR_ID),
                topMotor = ChargerSparkFlex(SHOOTER_MOTOR_ID){
                    ///// FOR NAYAN: Shooter configuration
                    inverted = true
                    smartCurrentLimit = SmartCurrentLimit(50.amps)
                },
                gearRatio = shooterRatio
            )
        }else{
            ShooterIOSim(
                DCMotorSim(DCMotor.getNeoVortex(1), shooterRatio, 0.004)
            )
        }
    )


    private val pivot = Pivot(
        if (isReal()){
            PivotIOReal(
                ChargerSparkMax(PIVOT_MOTOR_ID){
                    periodicFrameConfig = PeriodicFrameConfig.Optimized()
                    ///// FOR NAYAN: Change Current Limit for Pivot
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
                topMotor = ChargerSparkFlex(GROUND_INTAKE_ID),
                conveyorMotor = ChargerSparkMax(CONVEYOR_ID){
                    ///// FOR NAYAN: Ground Intake configuration
                    inverted = true
                    smartCurrentLimit = SmartCurrentLimit(35.amps)
                },
                intakeGearRatio = groundIntakeRatio
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
            ChargerSparkMax(DrivetrainID.TR_TURN),
            ChargerSparkMax(DrivetrainID.BL_TURN){ inverted = true },
            ChargerSparkMax(DrivetrainID.BR_TURN)
        ){
            periodicFrameConfig = PeriodicFrameConfig.Optimized()
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
            topLeftZero = 6.243.radians,
            topRightZero = 4.971.radians,
            bottomLeftZero = 1.37.radians,
            bottomRightZero = 0.621.radians,
        ),
        driveMotors = sparkMaxSwerveMotors(
            topLeft = ChargerSparkMax(DrivetrainID.TL_DRIVE){ inverted = true },
            topRight = ChargerSparkMax(DrivetrainID.TR_DRIVE),
            bottomLeft = ChargerSparkMax(DrivetrainID.BL_DRIVE){ inverted = true },
            bottomRight = ChargerSparkMax(DrivetrainID.BR_DRIVE)
        ){
            periodicFrameConfig = PeriodicFrameConfig.Optimized()
            smartCurrentLimit = SmartCurrentLimit(45.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        },
        controlData = SwerveControlData(
            azimuthControl = SwerveAzimuthControl.PID(
                PIDConstants(7.0,0,0.1),
            ),
            openLoopDiscretizationRate = 1.0,
            closedLoopDiscretizationRate = 0.0,
            velocityPID = PIDConstants(0.2,0.0,0.0),
            velocityFF = if (isReal()){
                AngularMotorFFEquation(0.16, 0.133)
            }else{
                AngularMotorFFEquation(0.0, 0.13)
            },
            robotRotationPID = PIDConstants(6.0, 0, 0.001),
            robotTranslationPID = PIDConstants(4.0,0,0.001)
        ),
        useOnboardPID = false,
        hardwareData = SwerveHardwareData.mk4iL2(
            turnMotorType = DCMotor.getNEO(1),
            driveMotorType = DCMotor.getNEO(1),
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



    private val autoChooser = AutoChooser(
        aprilTagVision = vision.tagPipeline,
        noteDetector = vision.notePipeline,
        drivetrain, shooter, pivot, groundIntake
    )

    override fun teleopInit(){
        vision.enableVisionPoseEstimation()
    }


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
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(
                DriverController.swerveOutput,
                fieldRelative = !DriverController.shouldDisableFieldRelative
            )
        }

        shooter.setDefaultRunCommand{
            val speed = OperatorInterface.shooterSpeedAxis()
            if (speed > 0.0){
                shooter.outtake(speed)
            }else{
                shooter.intake(speed)
            }
        }

        pivot.setDefaultRunCommand(endBehavior = pivot::setIdle){
            pivot.setSpeed(OperatorInterface.pivotSpeedAxis())
        }

        climber.setDefaultRunCommand{
            if (DriverController.climbersUpTrigger.asBoolean){
                moveLeftHook(1.0)
                moveRightHook(1.0)
            }else if (DriverController.climbersDownTrigger.asBoolean){
                moveLeftHook(-1.0)
                moveRightHook(-1.0)
            }else{
                moveLeftHook(0.0)
                moveRightHook(0.0)
            }
        }

        groundIntake.setDefaultRunCommand{
            groundIntake.setIdle()
        }
    }

    private fun configureBindings(){
        // used to make pivot side the front instead of the ground intake side
        val targetAngleOffset = 180.degrees

        fun resetAimToAngle() = runOnceCommand{
            drivetrain.removeRotationOverride()
        }

        fun targetAngle(heading: Angle) = runOnceCommand{
            val allianceAngleCompensation = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                180.degrees
            } else {
                0.degrees
            }

            drivetrain.setRotationOverride(
                AimToAngleRotationOverride(
                    heading + allianceAngleCompensation + targetAngleOffset,
                    ANGLE_TO_ROTATIONAL_VELOCITY_PID,
                    invert = isReal()
                )
            )
        }

        DriverController.apply{
            pointNorthTrigger.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())

            pointEastTrigger.onTrue(targetAngle(-90.degrees)).onFalse(resetAimToAngle())

            pointSouthTrigger.onTrue(targetAngle(-180.degrees)).onFalse(resetAimToAngle())

            pointWestTrigger.onTrue(targetAngle(-270.degrees)).onFalse(resetAimToAngle())

            zeroHeadingTrigger.onTrue(
                runOnceCommand{
                    gyroIO.zeroHeading(180.degrees)
                }
            )
        }

        // loopCommand and runOnceCommand are wrappers around RunCommand and InstantCommand
        // which allows for outer lambda block syntax
        OperatorInterface.apply{
            groundIntakeTrigger.whileTrue(runGroundIntake(groundIntake, shooter))

            groundOuttakeTrigger.whileTrue(loopCommand(groundIntake){ groundIntake.outtake() })

            passToShooterTrigger.whileTrue(passSerializedNote(groundIntake, shooter))

            //spinUpShooterTrigger.whileTrue(loopCommand(shooter){ shooter.outtakeAtSpeakerSpeed() })

            shootInSpeakerTrigger.whileTrue(
                shootInSpeaker(shooter, groundIntake, pivot, shooterSpinUpTime = 2.seconds)
            )

            // when only held, the buttons will just cause the pivot to PID to the appropriate position
            // interrupt behavior set as to prevent command scheduling conflicts
            ampScoreTrigger.whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.AMP)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            ampScoreTrigger.onClickAndHold(
                followPathOptimal(drivetrain, PathPlannerPath.fromPathFile("AmpTeleopNew"))
            )

            sourceIntakeLeftTrigger.or(sourceIntakeRightTrigger).whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.SOURCE)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )


            stowPivotTrigger.whileTrue(pivot.setAngleCommand(PivotAngle.STOWED))

            driveToNoteTrigger.whileTrue(
                pursueNoteElseTeleopDrive(drivetrain, vision.notePipeline)
                    .alongWith(runGroundIntake(groundIntake, shooter))
            )
        }
    }




    override val testCommand: Command get() = resetPoseThenFollowPath(
        drivetrain,
        PathPlannerPath.fromPathFile("RotationTest")
    )

        /*
        MechanicalAdvantageFFCharacterization(
            drivetrain, false,
            MechanicalAdvantageFFCharacterization.FeedForwardCharacterizationData("DrivetrainDataLeft"),
            MechanicalAdvantageFFCharacterization.FeedForwardCharacterizationData("DrivetrainDataRight"),
            { leftV, rightV ->
                drivetrain.setDriveVoltages(
                    listOf(
                        leftV.ofUnit(volts),
                        rightV.ofUnit(volts),
                        leftV.ofUnit(volts),
                        rightV.ofUnit(volts),
                    )
                )

                drivetrain.setTurnDirections(
                    listOf(
                        0.degrees, 0.degrees, 0.degrees, 0.degrees
                    )
                )
            },
            {
                val allVelocities = drivetrain.moduleAngularVelocities
                (allVelocities[0].siValue + allVelocities[2].siValue) / 2.0
            },
            {
                val allVelocities = drivetrain.moduleAngularVelocities
                (allVelocities[1].siValue + allVelocities[3].siValue) / 2.0
            }
        )
         */

    override val autonomousCommand: Command
        get() = if (isSimulation()) autoChooser.ampAutoTest else autoChooser.selected
}
