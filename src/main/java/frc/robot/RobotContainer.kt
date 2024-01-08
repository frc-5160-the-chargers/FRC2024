// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase.isReal

import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.constants.drivetrain.SwerveControlData
import frc.chargers.constants.drivetrain.SwerveHardwareData
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.configureIMUSimulation
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.commands.aimToApriltag
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController
import org.littletonrobotics.junction.Logger.recordOutput

class RobotContainer: ChargerRobotContainer() {


    private val gyroIO = ChargerNavX(useFusedHeading = false)

    private val limelight = ChargerLimelight(
        useJsonDump = false,
        lensHeight = LIMELIGHT_LENS_HEIGHT,
        mountAngle = LIMELIGHT_MOUNT_ANGLE
    )

    private val apriltagIO: VisionPipeline<VisionTarget.AprilTag> =
        limelight.ApriltagPipeline(
            index = 0,
            logInputs = LoggableInputsProvider("LimelightApriltagVision")
        )


    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            topLeftId = 29,
            topRightId = 31,
            bottomLeftId = 22,
            bottomRightId = 4
        ){
            smartCurrentLimit = SmartCurrentLimit(30.amps)
            voltageCompensationNominalVoltage = 12.volts
        },
        turnEncoders = swerveCANcoders(
            topLeftId = 44,
            topRightId = 42,
            bottomLeftId = 43,
            bottomRightId = 45,
            useAbsoluteSensor = true
        ).withOffsets(
            topLeftZero = 0.602.radians,
            topRightZero = 1.81.radians,
            bottomLeftZero = 1.48.radians,
            bottomRightZero = 2.936.radians
        ),
        driveMotors = sparkMaxSwerveMotors(
            topLeft = neoSparkMax(10){inverted = false},
            topRight = neoSparkMax(16){inverted = true},
            bottomLeft = neoSparkMax(30){inverted = false},
            bottomRight = neoSparkMax(3){inverted = false}
        ){
            smartCurrentLimit = SmartCurrentLimit(60.amps)
            voltageCompensationNominalVoltage = 12.volts
        },
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlData = SwerveControlData(
            anglePID = PIDConstants(10.0,0.0,0.0),
            velocityPID = PIDConstants.None,
            velocityFF = AngularMotorFFConstants.fromSI(0.0,0.0,0.0)
        ),
        hardwareData = SwerveHardwareData.mk4iL2(
            trackWidth = 32.inches,
            wheelBase = 32.inches
        ),
        gyro = if (isReal()) gyroIO else null,
    ).apply {



        defaultCommand = buildCommand{
            addRequirements(this@apply) // requires the drivetrain(the "this" of the apply function)

            if (isSimulation()){
                runOnce{
                    poseEstimator.zeroPose()
                }
            }

            var swerveOutput: ChassisPowers

            loopForever{
                swerveOutput = DriverController.swerveOutput(gyroIO.heading)
                swerveDrive(swerveOutput)
            }

            onEnd{
                stop()
            }
        }
    }



    init{
        if (DriverStationSim.getAllianceStationId() != AllianceStationID.Blue1){
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        }

        recordOutput("Tuning Mode", DashboardTuner.tuningMode)

        configureBindings()

        configureIMUSimulation(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )

        LiveWindow.disableAllTelemetry()
    }

    private fun configureBindings(){


        val resetAimToAngle = runOnceCommand{ DriverController.targetHeading = null}

        fun targetAngle(heading: Angle) = runOnceCommand(){ DriverController.targetHeading = heading}

        DriverController.apply{

            if (isReal()) {
                headingZeroButton.onTrue(InstantCommand(gyroIO::zeroHeading))
                poseZeroButton.onTrue(
                    runOnceCommand{
                        drivetrain.poseEstimator.zeroPose()
                        println("Pose has been reset.")
                    }
                )
            }



            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle)
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle)
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle)
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle)
        }

        OperatorController.apply{
            aimToTagButton.whileTrue(
                aimToApriltag(
                    pidConstants = AIM_TO_APRILTAG_PID,
                    drivetrain = drivetrain,
                    visionIO = apriltagIO
                )
            )
        }

    }




    override val autonomousCommand: Command
        get() = buildCommand(name = "Taxi Auto") {
            addRequirements(drivetrain)

            loopFor(3.seconds){
                drivetrain.swerveDrive(-0.3,0.0,0.0)
            }
        }

}
