package frc.robot.constants

import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.chargers.constants.SwerveControlData
import frc.chargers.controls.feedforward.AngularMotorFFConstants
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders

const val ODOMETRY_UPDATE_FREQUENCY_HZ = 250.0

val DRIVE_CONTROL_DATA = if (isReal()){
    SwerveControlData(
        anglePID = PIDConstants(4.0,0.0,0.0),
        velocityPID = PIDConstants(0.1,0.0,0.0),
        velocityFF = AngularMotorFFConstants.fromSI(0.00162,0.13394,0.0),
        robotRotationPID = PIDConstants(0.5, 0.0, 0.0), // for pathplanner
        robotTranslationPID = PIDConstants(0.5,0.0,0.0) // for pathplanner
    )
}else{
    SwerveControlData(
        anglePID = PIDConstants(10.0,0.0,0.0),
        velocityPID = PIDConstants(0.2,0.0,0.0),
        velocityFF = AngularMotorFFConstants.fromSI(0.12117,0.13210,0.0),
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
    topLeft = ChargerSparkMax(10){inverted = true},
    topRight = ChargerSparkMax(16){inverted = false},
    bottomLeft = ChargerSparkMax(30){inverted = true},
    bottomRight = ChargerSparkMax(3){inverted = true}
){
    smartCurrentLimit = SmartCurrentLimit(55.amps)
    voltageCompensationNominalVoltage = 12.volts
}