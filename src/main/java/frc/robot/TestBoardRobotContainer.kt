@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
import frc.chargers.hardware.sensors.vision.photonvision.ChargerPhotonCamera
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders

class TestBoardRobotContainer: ChargerRobotContainer() {

    // note of reference: an IO class is a low-level component of the robot
    // that integrates advantagekit logging.

    private val gyroIO = ChargerNavX(
        useFusedHeading = false,
        ahrs = AHRS(SPI.Port.kMXP, ODOMETRY_UPDATE_FREQUENCY_HZ.toInt().toByte())
    ).apply{ zeroHeading() }

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
            velocityFF = if (RobotBase.isReal()){
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
        gyro = if (RobotBase.isReal()) gyroIO else null,
    )


    private val testPhotonCam = ChargerPhotonCamera("Arducam_OV9281", UnitTransform3d())

    private val photonTagPipeline =
        testPhotonCam
            .AprilTagPipeline(0, LoggableInputsProvider("PhotonAprilTagPipeline"), usePoseEstimation = true)

    private val testLimelight = ChargerLimelight(robotToCamera = UnitTransform3d())

    private val limelightTagPipeline =
        testLimelight
            .AprilTagPipeline(0, LoggableInputsProvider("LimelightAprilTagPipeline"), usePoseEstimation = false)




    /*
    private val mlPipeline =
        testPhotonCam
            .ObjectPipeline(1, LoggableInputsProvider("MLTesting"))
     */

    val motor = TalonFX(7)

    override val autonomousCommand: Command
        get() = buildCommand {
            loopFor(3.seconds){
                println("running")
                motor.set(0.15)
            }
        }
}