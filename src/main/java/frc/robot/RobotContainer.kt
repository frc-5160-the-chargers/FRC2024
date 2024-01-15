// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

// part of Kmeasure
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.*

// WPILib imports
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand

// ChargerLib imports
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.constants.DashboardTuner
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.robot.commands.aimAndDriveToApriltag
import frc.robot.commands.aimToApriltag


//import frc.robot.commands.aimToApriltag
import frc.robot.commands.auto.pathplannerTaxi
import frc.robot.constants.*
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController

// AdvantageKit
import org.littletonrobotics.junction.Logger.recordOutput

class RobotContainer: ChargerRobotContainer() {


    // Subsystems/components

    private val gyroIO = ChargerNavX(useFusedHeading = false).apply{ zeroHeading() }


    private val limelight = ChargerLimelight(
        useJsonDump = false,
        lensHeight = LIMELIGHT_LENS_HEIGHT,
        mountAngle = LIMELIGHT_MOUNT_ANGLE
    )

    private val apriltagIO: VisionPipeline<VisionTarget.AprilTag> =
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
    ).apply {
        /*
        if (isReal()){
            poseEstimator = ThreadedPoseMonitor(
                OdometryIO(
                    this.hardwareData,
                    TURN_MOTORS,
                    TURN_ENCODERS,
                    DRIVE_MOTORS,
                    gyroIO
                ),
                kinematics = this.kinematics
            )
        }
         */

        IMUSimulation.configure(
            headingSupplier = { this.heading },
            chassisSpeedsSupplier = { this.currentSpeeds }
        )
    }


    init{
        if (DriverStationSim.getAllianceStationId() != AllianceStationID.Blue1){
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        }
        if (isSimulation()){
            DriverStation.silenceJoystickConnectionWarning(true)
        }
        recordOutput("Tuning Mode", DashboardTuner.tuningMode)
        LiveWindow.disableAllTelemetry()

        configureBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands(){
        drivetrain.defaultCommand = buildCommand{
            addRequirements(drivetrain)

            if (isSimulation()){
                runOnce{
                    drivetrain.poseEstimator.zeroPose()
                }
            }

            loop{
                drivetrain.swerveDrive(
                    DriverController.swerveOutput(drivetrain.heading)
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
            headingZeroButton.onTrue(InstantCommand(gyroIO::zeroHeading))
            poseZeroButton.onTrue(
                runOnceCommand{
                    drivetrain.poseEstimator.zeroPose()
                    println("Pose has been reset.")
                }
            )

            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle)
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle)
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle)
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle)
        }



        OperatorController.apply{
            aimToTagButton.whileTrue(
                aimToApriltag(
                    aimingPID = AIM_TO_APRILTAG_PID,
                    drivetrain = drivetrain,
                    visionIO = apriltagIO
                )
            )
            aimToTagAndDriveButton.whileTrue(
                aimAndDriveToApriltag(
                    wantedDistance = 5.inches,
                    targetId = 6,
                    distanceTargetPID = PIDConstants(0.2,0.0,0.0),
                    aimingPID = PIDConstants(0.2,0.0,0.0),
                    drivetrain = drivetrain,
                    visionIO = apriltagIO
                )
            )
        }
    }






    override val autonomousCommand: Command
        get() = pathplannerTaxi(drivetrain, "New Path")

}
