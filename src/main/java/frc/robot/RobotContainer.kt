// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.constants.drivetrain.SwerveControlData
import frc.chargers.constants.drivetrain.SwerveHardwareData
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.configureIMUSimulation
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.hardware.inputdevices.DriverController

class RobotContainer: ChargerRobotContainer() {

    val gyroIO = ChargerNavX(useFusedHeading = false)

    val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(0,1,2,3),
        turnEncoders = swerveCANcoders(0,1,2,3, useAbsoluteSensor = true),
        driveMotors = sparkMaxSwerveMotors(4,5,6,7),
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlData = SwerveControlData(
            anglePID = PIDConstants(10.0,0.0,0.0),
            velocityPID = PIDConstants.None,
            velocityFF = AngularMotorFFConstants.fromSI(0.0,0.0,0.0),
            openLoopDiscretizationRate = 2.3
        ),
        hardwareData = SwerveHardwareData.mk4iL2(
            trackWidth = 32.inches,
            wheelBase = 32.inches
        ),
        gyro = if (isReal()) gyroIO else null,
    ).apply{
        defaultCommand = buildCommand{
            addRequirements(this@apply)

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

        println("tuning mode: " + DashboardTuner.tuningMode)
        configureBindings()
        configureIMUSimulation(
            headingSupplier = { drivetrain.heading },
            chassisSpeedsSupplier = { drivetrain.currentSpeeds }
        )
    }

    private fun configureBindings(){
        val resetAimToAngle = InstantCommand{ DriverController.targetHeading = null}

        fun targetAngle(heading: Angle) = InstantCommand{ DriverController.targetHeading = heading}

        DriverController.apply{
            if (isReal()) {
                headingZeroButton.onTrue(InstantCommand(gyroIO::zeroHeading))
                poseZeroButton.onTrue(
                    InstantCommand{
                        drivetrain.poseEstimator.resetPose(UnitPose2d())
                        println("Pose has been reset.")
                    }
                )
            }

            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle)
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle)
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle)
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle)
        }

    }




    override val autonomousCommand: Command
        get() = buildCommand {
            runOnce{
                println("hi")
            }
        }

}
