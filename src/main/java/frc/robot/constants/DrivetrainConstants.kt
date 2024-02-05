package frc.robot.constants

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.motionprofiling.trapezoidal.AngularTrapezoidProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
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

val OPEN_LOOP_ROTATION_PID = PIDConstants(0.3,0,0)
val CLOSED_LOOP_ROTATION_PID = PIDConstants(0.3,0,0)

val OPEN_LOOP_TRANSLATION_PID = PIDConstants(0.3,0,0)
val CLOSED_LOOP_TRANSLATION_PID = PIDConstants(0.3,0,0)


val DRIVE_CONTROL_DATA = if (isReal()){
    SwerveControlData(
        azimuthControl = SwerveAzimuthControl.PID(
            PIDConstants(7.0,0.0,0.2),
            precision = Precision.Within(2.degrees)
        ),
        velocityPID = PIDConstants(0.1,0.0,0.0),
        velocityFF = AngularMotorFFEquation(0.12117,0.13210,0.0),
        robotRotationPID = CLOSED_LOOP_ROTATION_PID, // for pathplanner
        robotTranslationPID = CLOSED_LOOP_TRANSLATION_PID // for pathplanner
    )
}else{
    SwerveControlData(
        azimuthControl = SwerveAzimuthControl.ProfiledPID(
            PIDConstants(15.0,0,0.2),
            motionProfile = AngularTrapezoidProfile(
                maxVelocity = 13.0.radians / 1.seconds,
                maxAcceleration = 10.radians / 1.seconds / 1.seconds,
            )
        ),
        openLoopDiscretizationRate = 4.4,
        velocityPID = PIDConstants(0.2,0.0,0.0),
        velocityFF = AngularMotorFFEquation(0.12117,0.13210,0.0),
        robotRotationPID = CLOSED_LOOP_ROTATION_PID, // for pathplanner
        robotTranslationPID = CLOSED_LOOP_TRANSLATION_PID // for pathplanner
    )
}


val TURN_MOTORS = sparkMaxSwerveMotors(
    topLeft = ChargerSparkMax(29){inverted = true},
    topRight = ChargerSparkMax(31){inverted = true},
    bottomLeft = ChargerSparkMax(22),
    bottomRight = ChargerSparkMax(4){inverted = true}
){
    smartCurrentLimit = SmartCurrentLimit(30.amps)
    voltageCompensationNominalVoltage = 12.volts
    openLoopRampRate = 48.0
    closedLoopRampRate = 48.0
}.apply{
    forEach{
        it.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10)
    }
}

val TURN_ENCODERS = swerveCANcoders(
    topLeft = ChargerCANcoder(44){
         sensorDirection = SensorDirectionValue.Clockwise_Positive
    },
    topRight = ChargerCANcoder(42){
        sensorDirection = SensorDirectionValue.Clockwise_Positive
    },
    bottomLeft = ChargerCANcoder(43),
    bottomRight = ChargerCANcoder(45),
    useAbsoluteSensor = true
).withOffsets(
    topLeftZero = 0.973.radians,
    topRightZero = 2.881.radians,
    bottomLeftZero = 1.477.radians,
    bottomRightZero = 6.004.radians
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
}.apply{
    forEach{
        it.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10)
    }
}

