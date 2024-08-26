@file:Suppress("RedundantVisibilityModifier", "unused", "MemberVisibilityCanBePrivate", "LeakingThis")
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.dimensions.AngularVelocityDimension
import com.batterystaple.kmeasure.dimensions.VoltageDimension
import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
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
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.UnitPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.configuration.ConfigurableHardware
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.imu.HeadingProvider
import frc.chargers.hardware.sensors.imu.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.PoseEstimatingDrivetrain
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.angle
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import kotlin.jvm.optionals.getOrNull
import kotlin.reflect.full.primaryConstructor

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 *
 * Here, C represents the configuration of the motor, while M represents the motor controllers themselves.
 * Configure is a function that has the context of the configuration specified.
 */
inline fun <M, reified C : HardwareConfiguration> EncoderDifferentialDrivetrain(
    logName: String = "Drivetrain(Differential)",
    topLeft: M,
    topRight: M,
    bottomLeft: M,
    bottomRight: M,
    constants: DifferentialDriveConstants,
    gyro: HeadingProvider? = null,
    configure: C.() -> Unit
): EncoderDifferentialDrivetrain where M: Motor, M: ConfigurableHardware<C> {
    val primaryConstructor = C::class.primaryConstructor ?: error("The configuration class " + C::class.simpleName + " must have a constructor.")
    try{
        val configuration = primaryConstructor.callBy(emptyMap()).apply(configure)

        topLeft.configure(configuration)
        topRight.configure(configuration)
        bottomLeft.configure(configuration)
        bottomRight.configure(configuration)

        return EncoderDifferentialDrivetrain(logName, topLeft, topRight, bottomLeft, bottomRight, constants, gyro)
    }catch(e: Exception){
        error(
            "A configuration class must have a primary constructor with only default parameters; " +
            "however, " + C::class.simpleName + " does not."
        )
    }
}



public open class EncoderDifferentialDrivetrain(
    logName: String = "Drivetrain(Differential)",
    private val topLeft: Motor,
    private val topRight: Motor,
    private val bottomLeft: Motor,
    private val bottomRight: Motor,

    private val constants: DifferentialDriveConstants,
    val gyro: HeadingProvider? = null
): PoseEstimatingDrivetrain(logName), DifferentialDrivetrain, HeadingProvider {
    /* Private implementation */
    private val wheelRadius = constants.wheelDiameter / 2

    private val wheelTravelPerMotorRadian = wheelRadius / constants.gearRatio

    private val leftEncoder = topLeft.encoder
    private val rightEncoder = topRight.encoder

    private val startingLeftEncoderReading = leftEncoder.angularPosition
    private val startingRightEncoderReading = leftEncoder.angularPosition

    private val leftController = UnitPIDController<AngularVelocityDimension, VoltageDimension>(
        constants.velocityPID,
        outputRange = -12.volts..12.volts
    )
    private val rightController = UnitPIDController<AngularVelocityDimension, VoltageDimension>(
        constants.velocityPID,
        outputRange = -12.volts..12.volts
    )

    private var leftVoltage = 0.volts
    private var rightVoltage = 0.volts

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
            topRight.hasInvert = !topRight.hasInvert
            bottomRight.hasInvert = !bottomRight.hasInvert
        }

        when (constants.pathAlgorithm){
            DifferentialDriveConstants.PathAlgorithm.LTV -> {
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

            DifferentialDriveConstants.PathAlgorithm.RAMSETE -> {
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

    public val leftWheelTravel: Distance get() =
        (leftEncoder.angularPosition - startingLeftEncoderReading) * wheelTravelPerMotorRadian

    public val rightWheelTravel: Distance get() =
        (rightEncoder.angularPosition - startingRightEncoderReading) * wheelTravelPerMotorRadian

    /**
     * The total linear distance traveled since the start of the match.
     */
    public val distanceTraveled: Distance by logged{
        listOf(leftWheelTravel, rightWheelTravel).average()
    }

    /**
     * The current linear velocity of the robot.
     */
    public val velocity: Velocity by logged{
        listOf(
            leftEncoder.angularVelocity,
            rightEncoder.angularVelocity
        ).average() * wheelTravelPerMotorRadian
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
    public val currentSpeeds: ChassisSpeeds by logged(ChassisSpeeds.struct){
        kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(
                (leftEncoder.angularVelocity * wheelTravelPerMotorRadian).inUnit(meters/seconds),
                (rightEncoder.angularVelocity * wheelTravelPerMotorRadian).inUnit(meters/seconds),
            )
        )
    }

    // arcade drive and curvature drive implementations
    // are provided in the DifferentialDrivetrain interface
    override fun tankDrive(leftPower: Double, rightPower: Double) {
        leftVoltage = leftPower * 12.volts
        rightVoltage = rightPower * 12.volts
    }

    public fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): Unit =
        velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationSpeed))

    public fun velocityDrive(speeds: ChassisSpeeds){
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        velocityDrive(
            wheelSpeeds.leftMetersPerSecond.ofUnit(meters/seconds),
            wheelSpeeds.rightMetersPerSecond.ofUnit(meters/seconds)
        )
    }

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity) {
        val leftSetpoint = leftSpeed / wheelTravelPerMotorRadian
        val rightSetpoint = rightSpeed / wheelTravelPerMotorRadian

        leftVoltage = leftController.calculate(
            leftEncoder.angularVelocity / constants.gearRatio,
            leftSetpoint,
            constants.velocityFF.calculate(leftSetpoint)
        )
        rightVoltage = rightController.calculate(
            leftEncoder.angularVelocity / constants.gearRatio,
            rightSetpoint,
            constants.velocityFF.calculate(rightSetpoint)
        )
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

        topLeft.appliedVoltage = leftVoltage
        topRight.appliedVoltage = rightVoltage
        bottomLeft.appliedVoltage = leftVoltage
        bottomRight.appliedVoltage = rightVoltage
    }
}