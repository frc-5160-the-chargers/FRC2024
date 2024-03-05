@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.commands.runOnceCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
import frc.robot.hardware.inputdevices.DriverController
import kotlin.jvm.optionals.getOrNull

class TestBoardRobotContainer: ChargerRobotContainer() {

    private val gyroIO = ChargerNavX(useFusedHeading = false)

    private val testingDrivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            ChargerSparkMax(DrivetrainID.TL_TURN),
            ChargerSparkMax(DrivetrainID.TR_TURN),
            ChargerSparkMax(DrivetrainID.BL_TURN),
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
            topLeftZero = 0.973.radians,
            topRightZero = 2.881.radians,
            bottomLeftZero = 1.477.radians,
            bottomRightZero = 6.004.radians
        ),
        driveMotors = sparkMaxSwerveMotors(
            topLeft = ChargerSparkMax(DrivetrainID.TL_DRIVE),
            topRight = ChargerSparkMax(DrivetrainID.TR_DRIVE),
            bottomLeft = ChargerSparkMax(DrivetrainID.BL_DRIVE),
            bottomRight = ChargerSparkMax(DrivetrainID.BR_DRIVE)
        ){
            smartCurrentLimit = SmartCurrentLimit(45.amps)
            voltageCompensationNominalVoltage = 12.volts
            openLoopRampRate = 48.0
            closedLoopRampRate = 48.0
        },
        controlData = SwerveControlData(
            azimuthControl = SwerveAzimuthControl.PID(
                PIDConstants(12.0,0,0.2),
            ),
            openLoopDiscretizationRate = 4.5,
            velocityPID = PIDConstants(0.2,0.0,0.0),
            velocityFF = if (RobotBase.isReal()){
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
        gyro = if (RobotBase.isReal()) gyroIO else null,
    )

    init{
        testingDrivetrain.setDefaultRunCommand(endBehavior = {
            if (DriverStation.isEnabled()){
                println("Rumbling!")
                DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.2)
            }
        }){
            if (DriverController.shouldDisableFieldRelative){
                testingDrivetrain.swerveDrive(DriverController.swerveOutput, fieldRelative = false)
            }else{
                testingDrivetrain.swerveDrive(DriverController.swerveOutput)
            }
        }

        DriverController.apply{
            fun resetAimToAngle() = runOnceCommand{
                testingDrivetrain.removeRotationOverride()
            }

            fun targetAngle(heading: Angle) = runOnceCommand{
                val allianceAngleCompensation = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                    180.degrees
                } else {
                    0.degrees
                }

                testingDrivetrain.setRotationOverride(
                    AimToAngleRotationOverride(
                        heading + allianceAngleCompensation,
                        ANGLE_TO_ROTATIONAL_VELOCITY_PID,
                    )
                )
            }

            pointNorthTrigger.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())

            pointEastTrigger.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle())

            pointSouthTrigger.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle())

            pointWestTrigger.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle())
        }
    }


    override val autonomousCommand: Command
        get() = buildCommand {

        }
}