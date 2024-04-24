@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.robotposition.DifferentialPoseMonitor
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds

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
): SuperSubsystem(logName), DifferentialDrivetrain, HeadingProvider {

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

    private fun shouldFlipAlliance(): Boolean{
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        val alliance = DriverStation.getAlliance()
        return if (alliance.isPresent) {
            alliance.get() == DriverStation.Alliance.Red
        } else false
    }


    /* Public API */

    init{
        if (constants.invertMotors){
            topRight.hasInvert = !topRight.hasInvert
            bottomRight.hasInvert = !bottomRight.hasInvert
        }
        when (constants.pathAlgorithm){
            DifferentialDriveConstants.PathAlgorithm.LTV -> {
                AutoBuilder.configureLTV(
                    { poseEstimator.robotPose.inUnit(meters) },
                    { poseEstimator.resetPose(it.ofUnit(meters)) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                    constants.pathReplanningConfig,
                    ::shouldFlipAlliance,
                    this
                )
            }

            DifferentialDriveConstants.PathAlgorithm.RAMSETE -> {
                AutoBuilder.configureRamsete(
                    { poseEstimator.robotPose.inUnit(meters) },
                    { poseEstimator.resetPose(it.ofUnit(meters)) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    constants.pathReplanningConfig,
                    ::shouldFlipAlliance,
                    this
                )
            }
        }
    }

    /**
     * The pose estimator of the differential drivetrain.
     */
    public var poseEstimator: RobotPoseMonitor = DifferentialPoseMonitor(
        this,
        startingPose = startingPose
    )

    /**
     * The kinematics of the drivetrain.
     *
     * @see DifferentialDriveKinematics
     */
    public val kinematics: DifferentialDriveKinematics =
        DifferentialDriveKinematics(constants.width.inUnit(meters))

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
}