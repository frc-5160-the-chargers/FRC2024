@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
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
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.PoseEstimatingDrivetrain
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import kotlin.jvm.optionals.getOrNull

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 *
 * Here, C represents the configuration of the motor, while M represents the motor controllers themselves.
 */
inline fun <M, reified C : HardwareConfiguration> EncoderDifferentialDrivetrain(
    logName: String = "Drivetrain(Differential)",
    topLeft: M,
    topRight: M,
    bottomLeft: M,
    bottomRight: M,
    constants: DifferentialDriveConstants,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    configure: C.() -> Unit
): EncoderDifferentialDrivetrain where M: MotorizedComponent, M: HardwareConfigurable<C> {
    try{
        val configuration = C::class.constructors.first().call().apply{ configure() }

        topLeft.configure(configuration)
        topRight.configure(configuration)
        bottomLeft.configure(configuration)
        bottomRight.configure(configuration)

        return EncoderDifferentialDrivetrain(logName, topLeft, topRight, bottomLeft, bottomRight, constants, gyro, startingPose)
    }catch(e: Exception){
        error("It looks like your configuration class does not have a no-args constructor. This is not allowed.")
    }
}



@Suppress("MemberVisibilityCanBePrivate")
public class EncoderDifferentialDrivetrain(
    logName: String = "Drivetrain(Differential)",
    private val topLeft: MotorizedComponent,
    private val topRight: MotorizedComponent,
    private val bottomLeft: MotorizedComponent,
    private val bottomRight: MotorizedComponent,

    private val constants: DifferentialDriveConstants,
    val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d()
): PoseEstimatingDrivetrain(logName), DifferentialDrivetrain, HeadingProvider {

    /* Private implementation */
    private val wheelRadius = constants.wheelDiameter / 2

    private val wheelTravelPerMotorRadian = wheelRadius / constants.gearRatio

    private val leftEncoder = topLeft.encoder
    private val rightEncoder = topRight.encoder

    private val startingLeftEncoderReading = leftEncoder.angularPosition
    private val startingRightEncoderReading = leftEncoder.angularPosition

    private val leftController = SuperPIDController(
        constants.velocityPID,
        getInput = { leftEncoder.angularVelocity },
        target = AngularVelocity(0.0),
        feedforward = constants.velocityFF,
        selfSustain = true,
    )

    private val rightController = SuperPIDController(
        constants.velocityPID,
        getInput = { rightEncoder.angularVelocity },
        target = AngularVelocity(0.0),
        feedforward = constants.velocityFF,
        selfSustain = true,
    )

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

        resetPose(startingPose)

        when (constants.pathAlgorithm){
            DifferentialDriveConstants.PathAlgorithm.LTV -> {
                AutoBuilder.configureLTV(
                    { robotPose.inUnit(meters) },
                    { resetPose(it.ofUnit(meters)) },
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
                    { robotPose.inUnit(meters) },
                    { resetPose(it.ofUnit(meters)) },
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
    override val robotPose: UnitPose2d
        get() = poseEstimator.estimatedPosition.ofUnit(meters)

    /**
     * Resets the robot pose to a certain value.
     */
    override fun resetPose(pose: UnitPose2d) {
        if (gyro is ZeroableHeadingProvider){
            gyro.zeroHeading(pose.rotation)
            // uses pose rotation since gyro is going to be zeroed
            poseEstimator.resetPosition(
                pose.rotation.asRotation2d(),
                leftWheelTravel.siValue,
                rightWheelTravel.siValue,
                pose.inUnit(meters)
            )
        }else{
            poseEstimator.resetPosition(
                (gyro?.heading ?: this.heading).asRotation2d(),
                leftWheelTravel.siValue,
                rightWheelTravel.siValue,
                pose.inUnit(meters)
            )
        }
    }

    /**
     * Adds a vision measurement to the drivetrain's pose estimator.
     */
    override fun addVisionMeasurement(measurement: Measurement<UnitPose2d>, stdDevs: Matrix<N3, N1>?) {
        if (stdDevs != null){
            poseEstimator.addVisionMeasurement(
                measurement.value.inUnit(meters),
                measurement.timestamp.inUnit(seconds),
                stdDevs
            )
        }else{
            poseEstimator.addVisionMeasurement(
                measurement.value.inUnit(meters),
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
        topLeft.percentOut = leftPower
        bottomLeft.percentOut = leftPower
        topRight.percentOut = rightPower
        bottomRight.percentOut = rightPower
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

    public fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity){
        leftController.target = leftSpeed / wheelTravelPerMotorRadian
        rightController.target = rightSpeed / wheelTravelPerMotorRadian

        val leftV = leftController.calculateOutput()
        val rightV = rightController.calculateOutput()

        topLeft.appliedVoltage = leftV
        bottomLeft.appliedVoltage = leftV

        topRight.appliedVoltage = rightV
        bottomRight.appliedVoltage = rightV
    }


    override fun periodic(){
        poseEstimator.update(
            (gyro?.heading ?: this.heading).asRotation2d(),
            leftWheelTravel.siValue,
            rightWheelTravel.siValue,
        )
        robotObject.pose = poseEstimator.estimatedPosition
    }
}