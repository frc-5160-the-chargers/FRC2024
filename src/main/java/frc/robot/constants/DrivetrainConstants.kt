package frc.robot.constants

import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders

const val ODOMETRY_UPDATE_FREQUENCY_HZ = 250.0

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
    topLeft = neoSparkMax(10){inverted = false},
    topRight = neoSparkMax(16){inverted = true},
    bottomLeft = neoSparkMax(30){inverted = false},
    bottomRight = neoSparkMax(3){inverted = false}
){
    smartCurrentLimit = SmartCurrentLimit(60.amps)
    voltageCompensationNominalVoltage = 12.volts
}