package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.TimedRobot.isSimulation
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.hardware.motorcontrol.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.HeadingProvider
import frc.chargers.hardware.subsystems.swervedrive.*

fun getDrivetrain(gyro: HeadingProvider): EncoderHolonomicDrivetrain =
    if (isSimulation()) {
        EncoderHolonomicDrivetrain(
            turnMotors = SwerveData.create{ MotorSim(DCMotor.getNEO(1), moi = TURN_MOTOR_MOI) },
            turnEncoders = SwerveData.create { null },
            driveMotors = SwerveData.create{ MotorSim(DCMotor.getKrakenX60(1), moi = DRIVE_MOTOR_MOI) },
            constants = SWERVE_CONSTANTS
        )
    } else {
        EncoderHolonomicDrivetrain(turnMotors = TURN_MOTORS, turnEncoders = TURN_ENCODERS, driveMotors = DRIVE_MOTORS, constants = SWERVE_CONSTANTS, gyro = gyro)
    }

private object DrivetrainID {
    const val TL_DRIVE = 10
    const val TR_DRIVE = 3
    const val BL_DRIVE = 16
    const val BR_DRIVE = 30

    const val TL_TURN = 12
    const val TR_TURN = 4
    const val BL_TURN = 31
    const val BR_TURN = 22

    const val TL_ENCODER = 44
    const val TR_ENCODER = 45
    const val BL_ENCODER = 42
    const val BR_ENCODER = 43
}
private val SWERVE_CONSTANTS = SwerveConstants(
    moduleType = ModuleType.MK4iL2,
    trackWidth = 27.inches,
    wheelBase = 27.inches,
    azimuthPID = PIDConstants(7.0,0.0,0.0),
    azimuthPIDTolerance = 1.degrees,
    velocityPID = PIDConstants(0.05,0.0,0.0),
    velocityFF = AngularMotorFeedforward(0.0,0.13),
)
private val TURN_MOTORS = SwerveData(
    topLeft = ChargerSparkMax(DrivetrainID.TL_TURN),
    topRight = ChargerSparkMax(DrivetrainID.TR_TURN).configure(inverted = true),
    bottomLeft = ChargerSparkMax(DrivetrainID.BL_TURN),
    bottomRight = ChargerSparkMax(DrivetrainID.BR_TURN)
).map {
    it.configure(
        statorCurrentLimit = 30.amps,
        rampRate = 48.seconds,
        optimizeUpdateRate = true
    )
}
private val TURN_ENCODERS = SwerveData(
    // encoder - angleOffset = encoder with angle offset
    topLeft = ChargerCANcoder(DrivetrainID.TL_ENCODER) - 0.621.radians,
    topRight = ChargerCANcoder(DrivetrainID.TR_ENCODER) - 1.37.radians,
    bottomLeft = ChargerCANcoder(DrivetrainID.BL_ENCODER) - 4.971.radians,
    bottomRight = ChargerCANcoder(DrivetrainID.BR_ENCODER) - 6.243.radians
)
private val DRIVE_MOTORS = SwerveData(
    topLeft = ChargerTalonFX(DrivetrainID.TL_DRIVE),
    topRight = ChargerTalonFX(DrivetrainID.TR_DRIVE).configure(inverted = true),
    bottomLeft = ChargerTalonFX(DrivetrainID.BL_DRIVE),
    bottomRight = ChargerTalonFX(DrivetrainID.BR_DRIVE).configure(inverted = true),
).map {
    it.base.configurator.apply(
        CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(120.0)
    )
    it.configure(
        statorCurrentLimit = 60.amps,
        brakeWhenIdle = true,
        optimizeUpdateRate = true
    )
}
private val TURN_MOTOR_MOI = 0.004.kilo.grams * (meters * meters)
private val DRIVE_MOTOR_MOI = 0.025.kilo.grams * (meters * meters)