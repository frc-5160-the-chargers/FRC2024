// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.util.PathPlannerLogging
import com.revrobotics.CANSparkBase
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
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
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.PeriodicFrameConfig
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
import frc.robot.commands.*
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorInterface
import frc.robot.hardware.subsystems.climber.Climber
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIOReal
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIOReal
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIOSim
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIOReal
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIOSim
import frc.robot.hardware.subsystems.shooter.NoteVisualizer
import frc.robot.hardware.subsystems.shooter.Shooter
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOReal
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOSim
import org.littletonrobotics.junction.Logger.recordOutput
import kotlin.jvm.optionals.getOrNull


/**
 * The robot container for the official competition robot.
 */
class CompetitionRobotContainer: ChargerRobotContainer() {

    private val shooterRatio = 5.0 * (14.0 / 26.0) // torque * sprocket
    private val pivotRatio = 64.0
    private val groundIntakeRatio = 15.0 / 12.0
    private val conveyorRatio = 1.0

    // note of reference: an IO class is a low-level component of the robot
    // that integrates advantagekit logging.

    private val gyroIO = ChargerNavX(useFusedHeading = false).apply{ zeroHeading() }

    private val shooter = Shooter(
        if (isReal()){
            ShooterIOReal(
                topMotor = ChargerTalonFX(SHOOTER_MOTOR_ID){
                    ///// FOR NAYAN: Shooter configuration
                    inverted = true
                    statorCurrentLimitEnable = true
                    statorCurrentLimit = 65.amps
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
                    periodicFrameConfig = PeriodicFrameConfig.Custom(10, 10, 10, 10, 10, 10)
                    ///// FOR NAYAN: Change Current Limit for Pivot
                    smartCurrentLimit = SmartCurrentLimit(35.amps)
                },
                useOnboardPID = false,
                encoderType = PivotIOReal.EncoderType.IntegratedRelativeEncoder(pivotRatio),
                //offset = 1.503.radians
            )
        }else{
            PivotIOSim(DCMotorSim(DCMotor.getNEO(1), pivotRatio, 0.004))
        },
        ///// FOR NAYAN: Pivot PID Constants
        if (isReal()) PIDConstants(7.0,0,0)  else PIDConstants(10.0, 0.0, 0.0),
        AngularTrapezoidProfile(
            maxVelocity = AngularVelocity(8.0),
            maxAcceleration = AngularAcceleration(10.0)
        ),
        forwardSoftStop = 1.636.radians,
        //reverseSoftStop = (-1.576).radians
    )

    private val groundIntakeMotor = ChargerSparkFlex(GROUND_INTAKE_ID)

    private val groundIntake = GroundIntakeSerializer(
        if (isReal()){
            GroundIntakeIOReal(
                topMotor = groundIntakeMotor,
                conveyorMotor = ChargerSparkMax(CONVEYOR_ID){
                    ///// FOR NAYAN: Ground Intake configuration
                    inverted = true
                    smartCurrentLimit = SmartCurrentLimit(60.amps)
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

    private val climber = Climber(
        ClimberIOReal(
            ChargerSparkMax(CLIMBER_ID_LEFT){
                ///// FOR NAYAN: Climber Configuration
                inverted = true
                idleMode = CANSparkBase.IdleMode.kBrake
            },
            ChargerSparkMax(CLIMBER_ID_RIGHT){
                ///// FOR NAYAN: Climber Configuration
                idleMode = CANSparkBase.IdleMode.kBrake
            }
        )
    )


    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            ChargerSparkMax(DrivetrainID.TL_TURN),
            ChargerSparkMax(DrivetrainID.TR_TURN),
            ChargerSparkMax(DrivetrainID.BL_TURN){ inverted = true },
            ChargerSparkMax(DrivetrainID.BR_TURN)
        ){
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
            topLeftZero = 6.207.radians,
            topRightZero = 4.941.radians,
            bottomLeftZero = 1.4.radians,
            bottomRightZero = 0.661.radians,
        ),
        driveMotors = sparkMaxSwerveMotors(
            topLeft = ChargerSparkMax(DrivetrainID.TL_DRIVE){ inverted = true },
            topRight = ChargerSparkMax(DrivetrainID.TR_DRIVE),
            bottomLeft = ChargerSparkMax(DrivetrainID.BL_DRIVE){ inverted = true },
            bottomRight = ChargerSparkMax(DrivetrainID.BR_DRIVE)
        ){
            smartCurrentLimit = SmartCurrentLimit(45.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        },
        controlData = SwerveControlData(
            azimuthControl = SwerveAzimuthControl.PID(
                PIDConstants(7.0,0,0.1),
            ),
            openLoopDiscretizationRate = 4.5,
            velocityPID = PIDConstants(0.2,0.0,0.0),
            velocityFF = if (isReal()){
                AngularMotorFFEquation(0.12117,0.13210)
            }else{
                AngularMotorFFEquation(0.0081299, 0.13396)
            },
            robotRotationPID = PIDConstants(1.3,0,0.1),
            robotTranslationPID = PIDConstants(0.3,0,0.03)
        ),
        useOnboardPID = false,
        hardwareData = SwerveHardwareData.mk4iL2(
            turnMotorType = DCMotor.getNEO(1),
            driveMotorType = DCMotor.getNEO(1),
            maxModuleSpeed = 4.5.meters / 1.seconds,
            trackWidth = 32.inches, wheelBase = 32.inches
        ),
        gyro = if (isReal()) gyroIO else null,
    )

    //private val vision = VisionManager(drivetrain.poseEstimator, tunableCamerasInSim = false)



    private val autoChooser = AutoChooser(
        aprilTagVision = null,
        noteDetector = null,
        drivetrain, shooter, pivot, groundIntake
    )


    init{
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
        recordOutput("Tuning Mode", DashboardTuner.tuningMode)
        LiveWindow.disableAllTelemetry()

        configureBindings()
        configureDefaultCommands()

        IMUSimulation.configure(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )

        configurePathPlannerLogging()

        if (isSimulation()){
            NoteVisualizer.setRobotPoseSupplier { drivetrain.poseEstimator.robotPose }
            NoteVisualizer.setLauncherTransformSupplier { pivot.mechanism3dPose }
        }
    }

    private fun rumbleControllerOnInterrupt(){
        if (DriverStation.isEnabled()){
            println("Rumbling!")
            DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.2)
        }
    }

    private fun configureDefaultCommands(){
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(
                DriverController.swerveOutput,
                fieldRelative = !DriverController.shouldDisableFieldRelative
            )
        }

        shooter.setDefaultRunCommand(endBehavior = shooter::setIdle){
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
            if (DriverController.leftHookUpTrigger.asBoolean){
                climber.moveLeftHook(1.0)
            }else if (DriverController.leftHookDownTrigger.asBoolean){
                climber.moveLeftHook(-1.0)
            }else{
                climber.moveLeftHook(0.0)
            }

            if (DriverController.rightHookUpTrigger.asBoolean){
                climber.moveRightHook(1.0)
            }else if (DriverController.rightHookDownTrigger.asBoolean){
                climber.moveRightHook(-1.0)
            }else{
                climber.moveRightHook(0.0)
            }
        }
    }

    private fun configureBindings(){
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
                    heading + allianceAngleCompensation,
                    ANGLE_TO_ROTATIONAL_VELOCITY_PID,
                )
            )
        }

        DriverController.apply{
            pointNorthTrigger.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())

            pointEastTrigger.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle())

            pointSouthTrigger.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle())

            pointWestTrigger.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle())


        }

        OperatorInterface.apply{
            groundIntakeTrigger
                .whileTrue(runGroundIntake(groundIntake, pivot, shooter))
                .onFalse(loopCommand(groundIntake){ groundIntake.setIdle() })

            groundOuttakeTrigger
                .whileTrue(loopCommand(groundIntake){ groundIntake.outtake() })
                .onFalse(runOnceCommand(groundIntake){ groundIntake.setIdle() })

            passToShooterTrigger
                .whileTrue(passSerializedNote(groundIntake, shooter, pivot))
                .onFalse(loopCommand(groundIntake){ groundIntake.setIdle() })

            // when only held, the buttons will just cause the pivot to PID to the appropriate position
            // interrupt behavior set as to prevent command scheduling conflicts
            ampScoreTrigger.whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.AMP)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            sourceIntakeLeftTrigger.or(sourceIntakeRightTrigger).whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.SOURCE)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            stowPivotTrigger.whileTrue(pivot.setAngleCommand(PivotAngle.STOWED))

            // vision stuff commented out for now
            /*
            // when "double clicked" and held, the buttons will auto drive and move the pivot
            ampScoreTrigger.onClickAndHold(
                alignToAprilTag(
                    drivetrain,
                    vision.fusedTagPipeline,
                    pivot,
                    AprilTagLocation.AMP,
                    followPathCommand = followPathOptimal(
                        drivetrain,
                        PathPlannerPath.fromPathFile("AmpTeleop")
                    )
                )
            )

            sourceIntakeLeftTrigger.onClickAndHold(
                alignToAprilTag(
                    drivetrain,
                    vision.fusedTagPipeline,
                    pivot,
                    AprilTagLocation.SOURCE_LEFT,
                    followPathCommand = followPathOptimal(
                        drivetrain,
                        PathPlannerPath.fromPathFile("SourceLeftTeleop")
                    )
                )
            )

            sourceIntakeRightTrigger.onClickAndHold(
                alignToAprilTag(
                    drivetrain,
                    vision.fusedTagPipeline,
                    pivot,
                    AprilTagLocation.SOURCE_RIGHT,
                    followPathCommand = followPathOptimal(
                        drivetrain,
                        PathPlannerPath.fromPathFile("SourceRightTeleop")
                    )
                )
            )

            driveToNoteTrigger.whileTrue(
                pursueNoteElseTeleopDrive(drivetrain, vision.notePipeline)
            )

             */
        }
    }

    private fun configurePathPlannerLogging(){
        PathPlannerLogging.setLogCurrentPoseCallback {
            recordOutput("Pathplanner/currentPose", Pose2d.struct, it)
        }

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback {
            recordOutput("Pathplanner/targetPose", Pose2d.struct, it)

            val currPose = drivetrain.poseEstimator.robotPose.inUnit(meters)
            recordOutput("Pathplanner/deviationFromTargetPose/xMeters", it.x - currPose.x)
            recordOutput("Pathplanner/deviationFromTargetPose/yMeters", it.y - currPose.y)
            recordOutput("Pathplanner/deviationFromTargetPose/rotationRad", (it.rotation - currPose.rotation).radians)
        }
    }

    override val autonomousCommand: Command
        get() = autoChooser.selected


    override val testCommand: Command = FFCharacterize6328(
        drivetrain, true,
        FFCharacterize6328.FeedForwardCharacterizationData("DrivetrainDataLeft"),
        FFCharacterize6328.FeedForwardCharacterizationData("DrivetrainDataRight"),
        { leftV, rightV ->
            drivetrain.setDriveVoltages(
                listOf(
                    leftV.ofUnit(volts),
                    rightV.ofUnit(volts),
                    leftV.ofUnit(volts),
                    rightV.ofUnit(volts),
                )
            )
        },
        {
            val allVelocities = drivetrain.moduleAngularVelocities
            allVelocities[0].siValue + allVelocities[2].siValue / 2.0
        },
        {
            val allVelocities = drivetrain.moduleAngularVelocities
            allVelocities[1].siValue + allVelocities[3].siValue / 2.0
        }
    )
}
