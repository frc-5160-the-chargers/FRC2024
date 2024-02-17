// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.commands.commandbuilder.buildCommand
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
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkFlex
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
import frc.robot.commands.FieldLocation
import frc.robot.commands.auto.AutoChooser
import frc.robot.commands.driveToLocation
import frc.robot.commands.enableAimToSpeaker
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorInterface
import frc.robot.hardware.subsystems.groundintake.GroundIntake
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIOReal
import frc.robot.hardware.subsystems.groundintake.lowlevel.GroundIntakeIOSim
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIOReal
import frc.robot.hardware.subsystems.pivot.lowlevel.PivotIOSim
import frc.robot.hardware.subsystems.shooter.Shooter
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOReal
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOSim
import frc.robot.hardware.subsystems.vision.VisionManager
import org.littletonrobotics.junction.Logger.recordOutput


class CompetitionRobotContainer: ChargerRobotContainer() {

    private val shooterRatio = 5.0 * (14.0 / 26.0) // torque * sprocket
    private val pivotRatio = 64.0
    private val groundIntakeRatio = 15.0 / 12.0
    private val conveyorRatio = 1.0

    // note of reference: an IO class is a low-level component of the robot
    // that integrates advantagekit logging.

    private val gyroIO = ChargerNavX(
        useFusedHeading = false,
        ahrs = AHRS(SPI.Port.kMXP, ODOMETRY_UPDATE_FREQUENCY_HZ.toInt().toByte())
    ).apply{ zeroHeading() }

    private val shooter = Shooter(
        if (isReal()){
            ShooterIOReal(
                topMotor = ChargerSparkFlex(SHOOTER_ID_TOP),
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
                ChargerSparkMax(PIVOT_MOTOR_ID),
                ChargerCANcoder(PIVOT_ENCODER_ID),
                useOnboardPID = false,
                pivotRatio,
                0.degrees
            )
        }else{
            PivotIOSim(
                DCMotorSim(DCMotor.getNEO(1), pivotRatio, 0.010)
            )
        },
        PIDConstants(10.0,0,0),
        AngularTrapezoidProfile(
            maxVelocity = AngularVelocity(3.0),
            maxAcceleration = AngularAcceleration(5.0)
        )
    )

    private val groundIntake = GroundIntake(
        if (isReal()){
            GroundIntakeIOReal(
                ChargerSparkFlex(GROUND_INTAKE_ID),
                conveyorMotor = ChargerSparkMax(CONVEYOR_ID),
                conveyorVoltageMultiplier = 0.8,
                intakeGearRatio = groundIntakeRatio
            )
        }else{
            GroundIntakeIOSim(
                topMotorSim = DCMotorSim(DCMotor.getNeoVortex(1), groundIntakeRatio, 0.010),
                conveyorMotorSim = DCMotorSim(DCMotor.getNEO(1), conveyorRatio, 0.004)
            )
        }
    )

    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            ChargerSparkMax(DrivetrainID.TL_TURN){inverted = true},
            ChargerSparkMax(DrivetrainID.TR_TURN){inverted = true},
            ChargerSparkMax(DrivetrainID.BL_TURN),
            ChargerSparkMax(DrivetrainID.BR_TURN){inverted = true}
        ){
            smartCurrentLimit = SmartCurrentLimit(30.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        },
        turnEncoders = swerveCANcoders(
            topLeft = ChargerCANcoder(DrivetrainID.TL_ENCODER){ sensorDirection = SensorDirectionValue.Clockwise_Positive },
            topRight = ChargerCANcoder(DrivetrainID.TR_ENCODER){ sensorDirection = SensorDirectionValue.Clockwise_Positive },
            bottomLeft = ChargerCANcoder(DrivetrainID.BL_ENCODER),
            bottomRight = ChargerCANcoder(DrivetrainID.BR_ENCODER),
            useAbsoluteSensor = true
        ).withOffsets(
            topLeftZero = 0.973.radians,
            topRightZero = 2.881.radians,
            bottomLeftZero = 1.477.radians,
            bottomRightZero = 6.004.radians
        ),
        driveMotors = sparkMaxSwerveMotors(
            topLeft = ChargerSparkMax(DrivetrainID.TL_DRIVE){inverted = false},
            topRight = ChargerSparkMax(DrivetrainID.TR_DRIVE){inverted = true},
            bottomLeft = ChargerSparkMax(DrivetrainID.BL_DRIVE){inverted = false},
            bottomRight = ChargerSparkMax(DrivetrainID.BR_DRIVE){inverted = false}
        ){
            smartCurrentLimit = SmartCurrentLimit(45.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        },
        controlData = SwerveControlData(
            azimuthControl = SwerveAzimuthControl.ProfiledPID(
                PIDConstants(12.0,0,0.2),
                motionProfile = AngularTrapezoidProfile(
                    maxVelocity = 13.0.radians / 1.seconds,
                    maxAcceleration = 10.radians / 1.seconds / 1.seconds,
                )
            ),
            openLoopDiscretizationRate = 4.2,
            velocityPID = PIDConstants(0.2,0.0,0.0),
            velocityFF = if (isReal()){
                AngularMotorFFEquation(0.12117,0.13210)
            }else{
                AngularMotorFFEquation(0.0081299, 0.13396)
            },
            robotRotationPID = PIDConstants(1.2,0,0),
            robotTranslationPID = PIDConstants(1.2,0,0)
        ),
        useOnboardPID = false,
        hardwareData = SwerveHardwareData.mk4iL2(
            turnMotorType = DCMotor.getNEO(1),
            driveMotorType = DCMotor.getNEO(1),
            maxModuleSpeed = 4.5.meters / 1.seconds,
            maxModuleAcceleration = 25.meters / 1.seconds / 1.seconds,
            trackWidth = 32.inches, wheelBase = 32.inches
        ),
        gyro = if (isReal()) gyroIO else null,
    )

    private val vision = VisionManager(drivetrain.poseEstimator)




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

        configurePPLogging()

        AutoChooser.initOptions(
            vision.fusedTagPipeline, vision.notePipeline, drivetrain,
            shooter, pivot, groundIntake
        )
    }

    private fun configureDefaultCommands(){
        drivetrain.defaultCommand = buildCommand{
            addRequirements(drivetrain)

            loop{
                drivetrain.swerveDrive(DriverController.swerveOutput)
            }

            onEnd{
                drivetrain.stop()
            }
        }

        shooter.setDefaultRunCommand{
            val speed = OperatorInterface.rightY
            if (speed >= 0.0){
                shooter.outtake(speed)
            }else{
                shooter.intake(speed)
            }
        }

        pivot.setDefaultRunCommand{// function has context of pivot
            pivot.setSpeed(OperatorInterface.leftY / 6.0) // change this later
        }
    }

    private fun configureBindings(){
        fun resetAimToAngle() = runOnceCommand(drivetrain){
            drivetrain.removeRotationOverride()
        }

        fun targetAngle(heading: Angle) = runOnceCommand(drivetrain){
            drivetrain.setRotationOverride(
                AimToAngleRotationOverride(
                    heading,
                    ANGLE_TO_ROTATIONAL_VELOCITY_PID,
                )
            )
        }

        DriverController.apply{
            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle())
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle())
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle())
        }

        OperatorInterface.apply{
            val idleGroundIntake = runOnceCommand(groundIntake){ groundIntake.setIdle() }

            groundIntakeTrigger
                .whileTrue(loopCommand(groundIntake){ groundIntake.intake() })
                .onFalse(idleGroundIntake)

            groundOuttakeTrigger
                .whileTrue(loopCommand(groundIntake){ groundIntake.outtake() })
                .onFalse(idleGroundIntake)


            aimToSpeakerTrigger.whileTrue(enableAimToSpeaker(drivetrain, vision.fusedTagPipeline)).onFalse(
                runOnceCommand(drivetrain){
                    drivetrain.removeRotationOverride()
                }
            )

            driveToSourceLeftTrigger.whileTrue(
                driveToLocation(
                    FieldLocation.SOURCE_LEFT,
                    PathPlannerPath.fromPathFile("SourceLeftTeleop"),
                    drivetrain,
                    vision.fusedTagPipeline,
                    pivot
                )
            )

            driveToSourceRightTrigger.whileTrue(
                driveToLocation(
                    FieldLocation.SOURCE_RIGHT,
                    PathPlannerPath.fromPathFile("SourceRightTeleop"),
                    drivetrain,
                    vision.fusedTagPipeline,
                    pivot
                )
            )

            driveToAmpTrigger.whileTrue(
                driveToLocation(
                    FieldLocation.AMP,
                    PathPlannerPath.fromPathFile("AmpTeleop"),
                    drivetrain,
                    vision.fusedTagPipeline,
                    pivot
                )
            )
        }
    }

    private fun configurePPLogging(){
        PathPlannerLogging.setLogCurrentPoseCallback {
            recordOutput("Pathplanner/currentPose", Pose2d.struct, it)
        }

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback {
            //ChargerRobot.FIELD.getObject("PathplannerTargetPose").pose = it
            recordOutput("Pathplanner/targetPose", Pose2d.struct, it)

            val currPose = drivetrain.poseEstimator.robotPose.inUnit(meters)
            recordOutput("Pathplanner/deviationFromTargetPose/xMeters", it.x - currPose.x)
            recordOutput("Pathplanner/deviationFromTargetPose/yMeters", it.y - currPose.y)
            recordOutput("Pathplanner/deviationFromTargetPose/rotationRad", (it.rotation - currPose.rotation).radians)
        }
    }



    override val autonomousCommand: Command
        get() = AutoChooser.selected

    override val testCommand: Command get(){
        val routine = drivetrain.getDriveSysIdRoutine()
        return buildCommand {
            +routine.quasistatic(SysIdRoutine.Direction.kForward)
            +routine.quasistatic(SysIdRoutine.Direction.kReverse)
            +routine.dynamic(SysIdRoutine.Direction.kForward)
            +routine.dynamic(SysIdRoutine.Direction.kReverse)
        }
    }
}
