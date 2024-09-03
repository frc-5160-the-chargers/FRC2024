@file:Suppress("unused", "MemberVisibilityCanBePrivate", "LeakingThis")
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.imu.HeadingProvider
import frc.chargers.hardware.sensors.imu.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.PoseEstimatingDrivetrain
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.angle
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import kotlin.jvm.optionals.getOrNull

/**
 * A standard differential drive, with encoders.
 */
open class EncoderDifferentialDrivetrain(
    logName: String = "Drivetrain(Differential)",
    private val topLeft: Motor,
    private val topRight: Motor,
    private val bottomLeft: Motor,
    private val bottomRight: Motor,
    private val constants: DifferentialDriveConstants,
    val gyro: HeadingProvider? = null
): PoseEstimatingDrivetrain(logName), DifferentialDrivetrain, HeadingProvider {
    companion object {
        /**
         * Creates a [EncoderDifferentialDrivetrain] with simulated motors.
         */
        fun simulated(
            logName: String = "Drivetrain(Differential)",
            motorType: DCMotor,
            constants: DifferentialDriveConstants
        ): EncoderDifferentialDrivetrain =
            EncoderDifferentialDrivetrain(
                logName, MotorSim(motorType), MotorSim(motorType),
                MotorSim(motorType), MotorSim(motorType), constants
            )
    }
    /* Private implementation */
    private val wheelRadius = constants.wheelDiameter / 2

    private val leftEncoder = topLeft.encoder
    private val rightEncoder = topRight.encoder

    private val leftMotors = listOf(topLeft, bottomLeft)
    private val rightMotors = listOf(topRight, bottomRight)

    private val kinematics = DifferentialDriveKinematics(constants.width.inUnit(meters))
    private val poseEstimator = DifferentialDrivePoseEstimator(
        kinematics,
        Rotation2d(0.0),
        0.0, 0.0,
        Pose2d()
    )

    private val robotObject = ChargerRobot.FIELD.getObject(logName)

    /* Public API */
    init{
        if (constants.invertMotors){
            topRight.configure(inverted = !topRight.inverted)
            bottomRight.configure(inverted = !bottomRight.inverted)
        }
        for (motor in listOf(topLeft, topRight, bottomLeft, bottomRight)){
            motor.configure(
                velocityPID = constants.velocityPID,
                startingPosition = 0.degrees,
                gearRatio = constants.gearRatio
            )
        }

        when (constants.pathAlgorithm){
            PathAlgorithm.LTV -> {
                AutoBuilder.configureLTV(
                    { robotPose },
                    { resetPose(it) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                    constants.pathReplanningConfig,
                    { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }, // function used to determine if alliance flip is necessary
                    this
                )
            }

            PathAlgorithm.RAMSETE -> {
                AutoBuilder.configureRamsete(
                    { robotPose },
                    { resetPose(it) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    constants.pathReplanningConfig,
                    { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }, // function used to determine if alliance flip is necessary,
                    this
                )
            }
        }
    }

    val leftWheelTravel: Distance get() =
        leftEncoder.angularPosition * wheelRadius

    val rightWheelTravel: Distance get() =
        rightEncoder.angularPosition * wheelRadius

    /**
     * The total linear distance traveled since the start of the match.
     */
    val distanceTraveled: Distance by logged{
        listOf(leftWheelTravel, rightWheelTravel).average()
    }

    /**
     * The current linear velocity of the robot.
     */
    val velocity: Velocity by logged{
        listOf(
            leftEncoder.angularVelocity,
            rightEncoder.angularVelocity
        ).average() * wheelRadius
    }

    /**
     * The current heading (the direction the robot is facing).
     *
     * This value is calculated using the encoders, not a gyroscope or accelerometer,
     * so note that it may become inaccurate if the wheels slip. If available, consider
     * using a [frc.chargers.hardware.sensors.imu.ChargerNavX] or similar device to calculate heading instead.
     *
     * This value by itself is not particularly meaningful as it may be fairly large,
     * positive or negative, based on previous rotations of the motors, including
     * from previous times the robot has been enabled.
     *
     * Thus, it's more common to use this property to determine *change* in heading.
     * If the initial value of this property is stored, the amount of rotation since
     * that initial point can easily be determined by subtracting the initial heading
     * from the current heading.
     *
     * @see HeadingProvider
     */
    override val heading: Angle by logged{
        (leftWheelTravel - rightWheelTravel) / constants.width
    }

    /**
     * The current pose of the robot.
     */
    override val robotPose: Pose2d
        get() = poseEstimator.estimatedPosition

    /**
     * Resets the robot pose to a certain value.
     */
    override fun resetPose(pose: Pose2d) {
        if (gyro is ZeroableHeadingProvider){
            gyro.zeroHeading(pose.rotation.angle)
            // uses pose rotation since gyro is going to be zeroed
            poseEstimator.resetPosition(
                pose.rotation,
                leftWheelTravel.siValue,
                rightWheelTravel.siValue,
                pose
            )
        }else{
            poseEstimator.resetPosition(
                Rotation2d(gyro?.heading ?: this.heading),
                leftWheelTravel.siValue,
                rightWheelTravel.siValue,
                pose
            )
        }
    }

    /**
     * Adds a vision measurement to the drivetrain's pose estimator.
     */
    override fun addVisionMeasurement(measurement: Measurement<Pose2d>, stdDevs: Matrix<N3, N1>?) {
        if (stdDevs != null){
            poseEstimator.addVisionMeasurement(
                measurement.value,
                measurement.timestamp.inUnit(seconds),
                stdDevs
            )
        }else{
            poseEstimator.addVisionMeasurement(
                measurement.value,
                measurement.timestamp.inUnit(seconds)
            )
        }
    }

    /**
     * Gets the current [ChassisSpeeds] of the robot.
     */
    val currentSpeeds: ChassisSpeeds by logged(ChassisSpeeds.struct){
        kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(
                (leftEncoder.angularVelocity * wheelRadius).inUnit(meters / seconds),
                (rightEncoder.angularVelocity * wheelRadius).inUnit(meters / seconds),
            )
        )
    }

    // arcade drive and curvature drive implementations
    // are provided in the DifferentialDrivetrain interface
    override fun tankDrive(leftPower: Double, rightPower: Double) {
        leftMotors.forEach{ it.speed = leftPower }
        rightMotors.forEach{ it.speed = rightPower }
    }

    fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): Unit =
        velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationSpeed))

    fun velocityDrive(speeds: ChassisSpeeds){
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        velocityDrive(
            wheelSpeeds.leftMetersPerSecond.ofUnit(meters/seconds),
            wheelSpeeds.rightMetersPerSecond.ofUnit(meters/seconds)
        )
    }

    fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity) {
        leftMotors.forEach{
            it.setVelocitySetpoint(
                leftSpeed / wheelRadius,
                constants.velocityFF.calculate(leftSpeed / wheelRadius)
            )
        }
        rightMotors.forEach{
            it.setVelocitySetpoint(
                rightSpeed / wheelRadius,
                constants.velocityFF.calculate(rightSpeed / wheelRadius)
            )
        }
    }


    override fun periodic(){
        if (DriverStation.isDisabled()){
            stop()
        }
        poseEstimator.update(
            Rotation2d(gyro?.heading ?: this.heading),
            leftWheelTravel.siValue,
            rightWheelTravel.siValue,
        )
        robotObject.pose = poseEstimator.estimatedPosition
    }
}