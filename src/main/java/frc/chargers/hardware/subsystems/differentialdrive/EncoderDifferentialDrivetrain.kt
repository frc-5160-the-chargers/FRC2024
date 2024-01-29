@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.DiffDriveControlData
import frc.chargers.constants.DiffDriveHardwareData
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.configuration.HardwareConfigurable
import frc.chargers.hardware.configuration.HardwareConfiguration
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMaxConfiguration
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.differentialdrive.lowlevel.DiffDriveIO
import frc.chargers.hardware.subsystems.differentialdrive.lowlevel.DiffDriveIOReal
import frc.chargers.hardware.subsystems.differentialdrive.lowlevel.DiffDriveIOSim
import frc.chargers.hardware.subsystems.robotposition.DifferentialPoseMonitor
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger.recordOutput


private val standardLogInputs = LoggableInputsProvider(namespace = "Drivetrain(Differential)")


/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using SparkMax motor controllers.
 */
public inline fun sparkMaxDrivetrain(
    topLeft: ChargerSparkMax,
    topRight: ChargerSparkMax,
    bottomLeft: ChargerSparkMax,
    bottomRight: ChargerSparkMax,
    hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    controlData: DiffDriveControlData = DiffDriveControlData.None,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
    configure: ChargerSparkMaxConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        topLeft, topRight, bottomLeft, bottomRight, hardwareData, controlData,
        DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
        gyro, startingPose, *poseSuppliers,
        configuration = ChargerSparkMaxConfiguration().apply(configure)
    )


/**
 * A convenience function to create a [EncoderDifferentialDrivetrain]
 * using Talon FX motor controllers.
 */
public inline fun talonFXDrivetrain(
    topLeft: ChargerTalonFX,
    topRight: ChargerTalonFX,
    bottomLeft: ChargerTalonFX,
    bottomRight: ChargerTalonFX,
    hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    controlData: DiffDriveControlData = DiffDriveControlData.None,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
    configure: ChargerTalonFXConfiguration.() -> Unit = {}
): EncoderDifferentialDrivetrain =
    EncoderDifferentialDrivetrain(
        topLeft, topRight, bottomLeft, bottomRight, hardwareData, controlData,
        DifferentialDrivetrainSim.KitbotMotor.kDoubleFalcon500PerSide,
        gyro, startingPose, *poseSuppliers,
        configuration = ChargerTalonFXConfiguration().apply(configure)
    )

/**
 * A convenience function to create an [EncoderDifferentialDrivetrain]
 * allowing its motors to all be configured.
 *
 * Here, C represents the configuration of the motor, while M represents the motor controllers themselves.
 */
public fun <M, C : HardwareConfiguration> EncoderDifferentialDrivetrain(
    topLeft: M,
    topRight: M,
    bottomLeft: M,
    bottomRight: M,
    hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    controlData: DiffDriveControlData = DiffDriveControlData.None,
    simMotors: DifferentialDrivetrainSim.KitbotMotor = DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
    configuration: C
): EncoderDifferentialDrivetrain
    where M: EncoderMotorController, M: HardwareConfigurable<C> {
    val lowLevel = if (RobotBase.isReal()){
        DiffDriveIOReal(
            standardLogInputs,
            topLeft.apply{ configure(configuration) },
            topRight.apply{ configure(configuration) },
            bottomLeft.apply{ configure(configuration) },
            bottomRight.apply{ configure(configuration) }
        )
    }else{
        DiffDriveIOSim(standardLogInputs, simMotors)
    }

    return EncoderDifferentialDrivetrain(lowLevel, hardwareData, controlData, gyro, startingPose, *poseSuppliers)
}



@Suppress("MemberVisibilityCanBePrivate", "CanBeParameter")
public class EncoderDifferentialDrivetrain(
    lowLevel: DiffDriveIO,
    public val hardwareData: DiffDriveHardwareData = DiffDriveHardwareData.andyMark(),
    public val controlData: DiffDriveControlData = DiffDriveControlData.None,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
): SubsystemBase(), DifferentialDrivetrain, HeadingProvider, DiffDriveIO by lowLevel {

    /* Private implementation */
    private val wheelRadius = hardwareData.wheelDiameter / 2

    internal val wheelTravelPerMotorRadian = wheelRadius / hardwareData.gearRatio

    private val leftController = SuperPIDController(
        controlData.leftVelocityPID,
        getInput = {leftVelocity},
        target = AngularVelocity(0.0),
        feedforward = controlData.leftFF,
        selfSustain = true,
    )

    private val rightController = SuperPIDController(
        controlData.rightVelocityPID,
        getInput = {rightVelocity},
        target = AngularVelocity(0.0),
        feedforward = controlData.rightFF,
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
        inverted = hardwareData.invertMotors
        when (controlData.pathAlgorithm){
            DiffDriveControlData.PathAlgorithm.LTV -> {
                AutoBuilder.configureLTV(
                    { poseEstimator.robotPose.inUnit(meters) },
                    { poseEstimator.resetPose(it.ofUnit(meters)) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    ChargerRobot.LOOP_PERIOD.inUnit(seconds),
                    controlData.pathReplanConfig,
                    ::shouldFlipAlliance,
                    this
                )
            }

            DiffDriveControlData.PathAlgorithm.RAMSETE -> {
                AutoBuilder.configureRamsete(
                    { poseEstimator.robotPose.inUnit(meters) },
                    { poseEstimator.resetPose(it.ofUnit(meters)) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    controlData.pathReplanConfig,
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
        this, startingPose = startingPose, *poseSuppliers
    )

    /**
     * The kinematics of the drivetrain.
     *
     * @see DifferentialDriveKinematics
     */
    public val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
        hardwareData.width.inUnit(meters)
    )

    private val distanceOffset = a[leftWheelTravel,rightWheelTravel].average()

    /**
     * The total linear distance traveled since the start of the match.
     */
    public val distanceTraveled: Distance
        get() = (arrayOf(leftWheelTravel,rightWheelTravel).average() - distanceOffset) * wheelTravelPerMotorRadian

    /**
     * The current linear velocity of the robot.
     */
    public val velocity: Velocity
        get() = a[leftVelocity,rightVelocity].average() * wheelTravelPerMotorRadian

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
    public override val heading: Angle
        get() = wheelTravelPerMotorRadian * (rightWheelTravel - leftWheelTravel) / hardwareData.width

    /**
     * Gets the current [ChassisSpeeds] of the robot.
     */
    public val currentSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(
                (leftVelocity * wheelTravelPerMotorRadian).inUnit(meters/seconds),
                (rightVelocity * wheelTravelPerMotorRadian).inUnit(meters/seconds),
            )
        )

    // arcade drive and curvature drive implementations
    // are provided in the DifferentialDrivetrain interface
    override fun tankDrive(leftPower: Double, rightPower: Double) {
        setVoltages(leftPower * 12.volts, rightPower * 12.volts)
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

        setVoltages(
            left = leftController.calculateOutput(),
            right = rightController.calculateOutput()
        )
    }

    override fun periodic(){
        recordOutput("Drivetrain(Differential)/distanceTraveledMeters",distanceTraveled.inUnit(meters))
        recordOutput("Drivetrain(Differential)/calculatedHeadingRad", heading.inUnit(radians))
    }

}