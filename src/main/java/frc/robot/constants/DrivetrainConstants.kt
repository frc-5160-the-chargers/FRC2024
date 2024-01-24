package frc.robot.constants

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
import frc.chargers.pathplannerextensions.PathConstraints
import frc.chargers.utils.Precision

const val ODOMETRY_UPDATE_FREQUENCY_HZ = 200.0

val PATHFIND_CONSTRAINTS = PathConstraints(
    Velocity(2.0),
    Acceleration(2.0),
    AngularVelocity(2.0),
    AngularAcceleration(2.0)
)


val DRIVE_CONTROL_DATA = if (isReal()){
    SwerveControlData(
        azimuthControl = SwerveAzimuthControl.PID(
            PIDConstants(7.0,0.0,0.2),
            precision = Precision.Within(2.degrees)
        ),
        velocityPID = PIDConstants(0.1,0.0,0.0),
        velocityFF = AngularMotorFFEquation(0.12117,0.13210,0.0),
        robotRotationPID = PIDConstants(0.5, 0.0, 0.0), // for pathplanner
        robotTranslationPID = PIDConstants(0.5,0.0,0.0) // for pathplanner
    )
}else{
    SwerveControlData(
        azimuthControl = SwerveAzimuthControl.ProfiledPID(
             PIDConstants(13.0,0,0.2),
            motionProfile = AngularTrapezoidProfile(
                maxVelocity = 7.0.radians / 1.seconds,
                maxAcceleration = 4.radians / 1.seconds / 1.seconds,
            )
        ),
        velocityPID = PIDConstants(0.2,0.0,0.0),
        velocityFF = AngularMotorFFEquation(0.12117,0.13210,0.0),
        robotRotationPID = PIDConstants(0.5, 0.0, 0.0), // for pathplanner
        robotTranslationPID = PIDConstants(0.5,0.0,0.0) // for pathplanner
    )
}


val TURN_MOTORS = sparkMaxSwerveMotors(
    topLeftId = 29,
    topRightId = 31,
    bottomLeftId = 22,
    bottomRightId = 4
){
    smartCurrentLimit = SmartCurrentLimit(30.amps)
    voltageCompensationNominalVoltage = 12.volts
    openLoopRampRate = 48.0
    closedLoopRampRate = 48.0
}

val TURN_ENCODERS = swerveCANcoders(
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
)

val DRIVE_MOTORS = sparkMaxSwerveMotors(
    topLeft = ChargerSparkMax(10){inverted = false},
    topRight = ChargerSparkMax(16){inverted = true},
    bottomLeft = ChargerSparkMax(30){inverted = false},
    bottomRight = ChargerSparkMax(3){inverted = false}
){
    smartCurrentLimit = SmartCurrentLimit(45.amps)
    voltageCompensationNominalVoltage = 12.volts
    openLoopRampRate = 48.0
    closedLoopRampRate = 48.0
}

