// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.constants.DashboardTuner
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.*
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.subsystems.odometry.ThreadedPoseMonitor
import org.littletonrobotics.junction.Logger.hasReplaySource
import org.littletonrobotics.junction.Logger.recordOutput


class RobotContainer: ChargerRobotContainer() {

    // Subsystems/components

    private val gyroIO = ChargerNavX(
        useFusedHeading = false,
        ahrs = AHRS(SPI.Port.kMXP, ODOMETRY_UPDATE_FREQUENCY_HZ.toInt().toByte())
    ).apply{ zeroHeading() }


    private val limelight = ChargerLimelight(
        useJsonDump = false,
        lensHeight = LIMELIGHT_LENS_HEIGHT,
        mountAngle = LIMELIGHT_MOUNT_ANGLE
    )

    /*
    private val apriltagIO: AprilTagVisionPipeline =
        limelight.ApriltagPipeline(
            index = 1,
            logInputs = LoggableInputsProvider("LimelightApriltagVision") // logs to the "LimelightApriltagVision" namespace
        )

     */

    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = TURN_MOTORS,
        turnEncoders = TURN_ENCODERS,
        driveMotors = DRIVE_MOTORS,
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlData = DRIVE_CONTROL_DATA,
        hardwareData = SwerveHardwareData.mk4iL2(trackWidth = 32.inches, wheelBase = 32.inches),
        gyro = if (isReal()) gyroIO else null,
    )

    /*
    private val shooter = Shooter(
        ShooterIOSim(
            DCMotor.getNEO(1),
            DCMotor.getNEO(1),
            DCMotor.getNEO(1), // sim motors,
            1.0, 1.0, 1.0
        ),
        PIDConstants(0.3,0,0)
    )

     */


    init{
        if (DriverStationSim.getAllianceStationId() != AllianceStationID.Blue1){
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        }
        DriverStation.silenceJoystickConnectionWarning(true)
        recordOutput("Tuning Mode", DashboardTuner.tuningMode)
        LiveWindow.disableAllTelemetry()

        configureBindings()
        configureDefaultCommands()

        /*
        /*
        PPHolonomicDriveController.setRotationTargetOverride {
            Optional.of(Rotation2d.fromDegrees(90.0))
        }

         */
        IMUSimulation.configure(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )

         */


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


        /*
        PathPlannerLogging.setLogCurrentPoseCallback {
            recordOutput("Pathplanner/currentPose", Pose2d.struct, it)
        }

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback {
            ChargerRobot.FIELD.getObject("PathplannerTargetPose").pose = it
            recordOutput("Pathplanner/targetPose", Pose2d.struct, it)

            val currPose = drivetrain.poseEstimator.robotPose.inUnit(meters)
            recordOutput("Pathplanner/deviationFromTargetPose/xMeters", it.x - currPose.x)
            recordOutput("Pathplanner/deviationFromTargetPose/yMeters", it.y - currPose.y)
            recordOutput("Pathplanner/deviationFromTargetPose/rotationRad", (it.rotation - currPose.rotation).radians)
        }

         */
    }

    private fun configureDefaultCommands(){

        drivetrain.defaultCommand = buildCommand{
            addRequirements(drivetrain)

            loop{
                drivetrain.swerveDrive(
                    DriverController.swerveOutput(drivetrain.heading),
                    fieldRelative = true
                )
            }

            onEnd{
                drivetrain.stop()
            }
        }
    }


    private fun configureBindings(){

        val resetAimToAngle = runOnceCommand{ DriverController.targetHeading = null}

        fun targetAngle(heading: Angle) = runOnceCommand{ DriverController.targetHeading = heading }

        DriverController.apply{
            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle)
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle)
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle)
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle)
        }

    }


    override val autonomousCommand: Command
        get() = runOnceCommand{}



    /*
    val autoChooser = LoggedDashboardChooser<Command>("Auto Command").apply{
        addDefaultOption("Taxi", basicTaxi(drivetrain))
    }


    override val autonomousCommand: Command
        get() = buildCommand {
            loopFor(3.seconds, drivetrain){
                drivetrain.velocityDrive(Velocity(1.0), Velocity(0.0), AngularVelocity(0.0))
            }




            val testPath = PathPlannerPath.fromPathFile("New Path")

            runOnce(drivetrain){
                drivetrain.poseEstimator.resetPose(testPath.previewStartingHolonomicPose.ofUnit(meters))
            }

            +AutoBuilder.followPath(testPath)
        }

    override val testCommand: Command
        get() = runOnceCommand(drivetrain){
            drivetrain.poseEstimator.zeroPose()
            println("Pose zeroed. New pose: " + drivetrain.poseEstimator.robotPose)
            gyroIO.zeroHeading()
            println("Gyro zeroed. New gyro heading: " + gyroIO.heading)
        }

     */

    /*
    val motor = ChargerSparkMax(29).apply{
        this.getEncoder().position = 0.0
        restoreFactoryDefaults()
    }
    override val autonomousCommand: Command
        get() = buildCommand {
            loop{
                /*
                motor.setAngularPosition(
                    90.degrees,
                    PIDConstants(0.3,0,0),
                )
                 */
                motor.set(0.3)
                recordOutput("Testing/motorPosition", motor.encoder.angularPosition.inUnit(degrees))
            }
        }

     */

}
