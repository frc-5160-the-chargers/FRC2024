package frc.debugging

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.framework.ChargerRobotContainer

class EmptyRobotContainer: ChargerRobotContainer() {
    override val autonomousCommand: Command = InstantCommand()

    /*
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
            velocityFF = if (RobotBase.isReal()){
                AngularMotorFFEquation(0.2523, 0.08909)
            }else{
                AngularMotorFFEquation(0.0081299, 0.13396)
            },
            robotRotationPID = PIDConstants(1.3,0,0.1),
            robotTranslationPID = PIDConstants(0.8,0,0.03)
        ),
        useOnboardPID = false,
        hardwareData = SwerveHardwareData.mk4iL2(
            turnMotorType = DCMotor.getNEO(1),
            driveMotorType = DCMotor.getNEO(1),
            maxModuleSpeed = 4.5.meters / 1.seconds,
            trackWidth = 32.inches, wheelBase = 32.inches
        ),
        gyro = null,
        invertPoseX = RobotBase.isReal(),
        invertPoseY = RobotBase.isReal()
    )

    init{
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(
                DriverController.swerveOutput,
                fieldRelative = !DriverController.shouldDisableFieldRelative
            )
        }
        DriverStation.silenceJoystickConnectionWarning(true)
    }

     */


}