// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.constants.DashboardTuner
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.constants.*
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.subsystems.shooter.Shooter
import frc.robot.hardware.subsystems.shooter.lowlevel.ShooterIOSim
import org.littletonrobotics.junction.Logger.recordOutput


class RobotContainer: ChargerRobotContainer() {



    private val gyroIO = ChargerNavX(
        useFusedHeading = false,
        ahrs = AHRS(SPI.Port.kMXP, ODOMETRY_UPDATE_FREQUENCY_HZ.toInt().toByte())
    ).apply{ zeroHeading() }


    /*
    private val limelight = ChargerLimelight(
        useJsonDump = false,
        lensHeight = LIMELIGHT_LENS_HEIGHT,
        mountAngle = LIMELIGHT_MOUNT_ANGLE
    )

    private val apriltagIO: AprilTagVisionPipeline =
        limelight.ApriltagPipeline(
            index = 1,
            logInputs = LoggableInputsProvider("LimelightApriltagVision") // logs to the "LimelightApriltagVision" namespace
        )
     */

    private val shooter = Shooter(
        ShooterIOSim(
            DCMotor.getNEO(1),
            DCMotor.getNEO(1),
            1.0, 1.0
        ),
        PIDConstants(0.3,0,0),
    )


    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = TURN_MOTORS,
        turnEncoders = TURN_ENCODERS,
        driveMotors = DRIVE_MOTORS,
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlData = DRIVE_CONTROL_DATA,
        useOnboardPID = false,
        hardwareData = SwerveHardwareData.mk4iL2(
            maxModuleSpeed = 4.5.meters / 1.seconds,
            maxModuleAcceleration = 25.meters / 1.seconds / 1.seconds,
            trackWidth = 32.inches, wheelBase = 32.inches
        ),
        gyro = if (isReal()) gyroIO else null,
    )


    init{
        println(DriverStationSim.getAllianceStationId())

        if (DriverStationSim.getAllianceStationId() != AllianceStationID.Blue1){
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        }



        println(DriverStationSim.getAllianceStationId())
        DriverStation.silenceJoystickConnectionWarning(true)
        DashboardTuner.tuningMode = true
        recordOutput("Tuning Mode", DashboardTuner.tuningMode)
        LiveWindow.disableAllTelemetry()

        configureBindings()
        configureDefaultCommands()



        IMUSimulation.configure(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )



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

    }

    private fun configureBindings(){


        fun resetAimToAngle() = runOnceCommand(drivetrain){
            drivetrain.removeRotationOverride()
        }

        fun targetAngle(heading: Angle) = runOnceCommand(drivetrain){
            drivetrain.setRotationOverride(
                AimToAngleRotationOverride(
                    heading,
                    PIDConstants(3.5,0,0),
                )
            )
        }

        DriverController.apply{
            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle())
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle())
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle())
        }

    }

    override val autonomousCommand: Command
        get() = buildCommand {
            /*
            val motor = TURN_MOTORS.topLeft

            loop(drivetrain){
                motor.setAngularPosition(
                    90.degrees,
                    PIDConstants(0.3,0,0),
                    turnEncoder = TURN_ENCODERS.topLeft,
                    motorToEncoderRatio = 150.0 / 7.0,
                    continuousWrap = true,
                )
            }

             */



            /*
            val trajGroupName = "5pAutoLeft"

            runOnce {
                drivetrain.poseEstimator.resetToChoreoTrajectory(trajGroupName)
            }

            val paths = PathPlannerPaths.fromChoreoTrajectoryGroup(trajGroupName)

            paths.forEach{
                +AutoBuilder.followPath(it)
            }

             */


            runOnce(drivetrain){
                drivetrain.poseEstimator.zeroPose()
            }

            loopFor(Time(3.0), drivetrain){
                drivetrain.velocityDrive(Velocity(2.0), Velocity(0.0), AngularVelocity(0.0))
            }

            loopFor(Time(3.0), drivetrain){
                drivetrain.stop()
            }

            runOnce(drivetrain){
                drivetrain.poseEstimator.zeroPose()
            }

            loopFor(Time(3.0), drivetrain){
                drivetrain.velocityDrive(Velocity(2.0), Velocity(2.0), AngularVelocity(0.0))
            }

            loop(drivetrain){
                drivetrain.stop()
            }
        }

    override val testCommand: Command =
        drivetrain.getDriveSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward)
}
