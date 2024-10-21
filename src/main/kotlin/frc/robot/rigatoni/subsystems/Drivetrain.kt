package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.*
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
            name = "SwerveDrive",
            turnMotors = List(4){ MotorSim(DCMotor.getNEO(1), moi = TURN_MOTOR_MOI) },
            driveMotors = List(4){ MotorSim(DCMotor.getKrakenX60(1), moi = DRIVE_MOTOR_MOI) },
            constants = SWERVE_CONSTANTS
        )
    } else {
        EncoderHolonomicDrivetrain(
            name = "SwerveDrive", turnMotors = TURN_MOTORS, turnEncoders = TURN_ENCODERS,
            driveMotors = DRIVE_MOTORS, constants = SWERVE_CONSTANTS, gyro = gyro
        )
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
    moduleType = ModuleType.Mk4iL2,
    trackWidth = 27.inches,
    wheelBase = 27.inches,
    azimuthPID = PIDConstants(2.0,0.0,0.0),
    azimuthPIDTolerance = 1.degrees,
    velocityPID = PIDConstants(0.05,0.0,0.0),
    velocityFF = AngularMotorFeedforward(0.0,0.13),
)
private val TURN_MOTORS = listOf(
    ChargerSparkMax(DrivetrainID.TL_TURN),
    ChargerSparkMax(DrivetrainID.TR_TURN).configure(inverted = true),
    ChargerSparkMax(DrivetrainID.BL_TURN),
    ChargerSparkMax(DrivetrainID.BR_TURN)
).map {
    it.configure(
        statorCurrentLimit = 30.amps,
        rampRate = 48.seconds,
        optimizeUpdateRate = true
    )
}
private val TURN_ENCODERS = listOf(
    // encoder - angleOffset = encoder with angle offset
    ChargerCANcoder(DrivetrainID.TL_ENCODER) + 2.447.radians,
    ChargerCANcoder(DrivetrainID.TR_ENCODER) - 1.379.radians,
    ChargerCANcoder(DrivetrainID.BL_ENCODER) + 1.312.radians,
    ChargerCANcoder(DrivetrainID.BR_ENCODER) - 3.116.radians
)
private val DRIVE_MOTORS = listOf(
    ChargerTalonFX(DrivetrainID.TL_DRIVE),
    ChargerTalonFX(DrivetrainID.TR_DRIVE),
    ChargerTalonFX(DrivetrainID.BL_DRIVE),
    ChargerTalonFX(DrivetrainID.BR_DRIVE).configure(inverted = true),
).map {
    it.configure(
        statorCurrentLimit = 60.amps,
        brakeWhenIdle = true,
        optimizeUpdateRate = true
    ).limitSupplyCurrent(
        45.amps,
        highLimit = 70.amps,
        highLimitAllowedFor = 0.2.seconds
    )
}
private val TURN_MOTOR_MOI = 0.004.kilo.grams * (meters * meters)
private val DRIVE_MOTOR_MOI = 0.025.kilo.grams * (meters * meters)