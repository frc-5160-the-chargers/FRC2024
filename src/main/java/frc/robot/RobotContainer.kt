// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.seconds
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.constants.DashboardTuner
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.delay
import frc.robot.constants.*
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.subsystems.odometry.OdometryIO
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

    private val apriltagIO: AprilTagVisionPipeline =
        limelight.ApriltagPipeline(
            index = 1,
            logInputs = LoggableInputsProvider("LimelightApriltagVision") // logs to the "LimelightApriltagVision" namespace
        )

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
        PPHolonomicDriveController.setRotationTargetOverride {
            Optional.of(Rotation2d.fromDegrees(90.0))
        }

         */
        IMUSimulation.configure(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )

        // waits 0.02 seconds to prevent potential issues
        delay(0.02.seconds)

        if (isReal() || hasReplaySource()){
            drivetrain.poseEstimator = ThreadedPoseMonitor(
                OdometryIO(
                    drivetrain.hardwareData,
                    TURN_MOTORS,
                    TURN_ENCODERS,
                    DRIVE_MOTORS,
                    gyroIO
                ),
                kinematics = drivetrain.kinematics
            )
        }
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
        get() = buildCommand {
            loop(drivetrain){
                drivetrain.swerveDrive(0.8,0.0,0.06)
            }
        }

    override val testCommand: Command
        get() = runOnceCommand(drivetrain){
            drivetrain.poseEstimator.zeroPose()
            println("Pose zeroed. New pose: " + drivetrain.poseEstimator.robotPose)
            gyroIO.zeroHeading()
            println("Gyro zeroed. New gyro heading: " + gyroIO.heading)
        }

}
