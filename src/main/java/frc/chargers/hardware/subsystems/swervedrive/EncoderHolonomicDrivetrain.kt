@file:Suppress("RedundantVisibilityModifier", "unused", "MemberVisibilityCanBePrivate")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.SuperSubsystem
import frc.chargers.hardware.motorcontrol.MotorizedComponent
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.hardware.subsystems.robotposition.SwervePoseMonitor
import frc.chargers.pathplannerextensions.asPathPlannerConstants
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.math.units.VoltageRate
import frc.chargers.utils.math.units.toKmeasure
import frc.chargers.utils.math.units.toWPI
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.chargers.wpilibextensions.kinematics.*
import frc.external.frc6328.SwerveSetpointGenerator
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow


/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 *
 * This class implements the DifferentialDrivetrain interface for basic utility use
 * and interop with existing DifferentialDrive extension functions.
 *
 * Note: TrackWidth is the horizontal length of the robot, while wheelBase is the vertical length of the robot.
 */
public class EncoderHolonomicDrivetrain(
    logName: String = "Drivetrain(Swerve)",
    turnMotors: SwerveData<MotorizedComponent>,
    // turn encoders are optional in sim
    turnEncoders: SwerveData<PositionEncoder?> = SwerveData.create{ null },
    driveMotors: SwerveData<MotorizedComponent>,
    private val chassisConstants: SwerveChassisConstants,
    private val moduleConstants: SwerveModuleConstants,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(Distance(0.0), Distance(0.0), gyro?.heading ?: Angle(0.0)),
): SuperSubsystem(logName), HeadingProvider {

    private val moduleNames = listOf("Modules/TopLeft", "Modules/TopRight", "Modules/BottomLeft", "Modules/BottomRight")
    /** A [SwerveData] instance that holds all the swerve modules of the drivetrain. */
    private val swerveModules: SwerveData<SwerveModule> =
        SwerveData.create{ index ->
            SwerveModule(
                namespace = logName + "/" + moduleNames[index],
                turnMotors[index],
                turnEncoders[index],
                driveMotors[index],
                moduleConstants
            )
        }

    private val constraints = SwerveSetpointGenerator.ModuleLimits(
        moduleConstants.driveMotorMaxSpeed.siValue,
        moduleConstants.driveMotorMaxAcceleration.siValue,
        moduleConstants.turnMotorMaxSpeed.siValue
    )
    private val allianceFieldRelativeOffset: Angle
        get() = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
            180.degrees
        }else{
            0.degrees
        }

    /** An enum class that stores the current control mode of the drivetrain. */
    private enum class ControlMode{
        OPEN_LOOP,  // Represents open-loop control with no feedforward / PID
        CLOSED_LOOP, // Represents closed-loop control with feedforward & PID: this is far more accurate than open loop in terms of velocity
        NONE // Represents no control at all; this mode should be set when the drivetrain is not calling one of the drive functions.
    }

    private var rotationOverride: RotationOverride? = null
    private var currentControlMode: ControlMode = ControlMode.NONE
    private var goal: ChassisSpeeds = ChassisSpeeds()
    private var setpoint: SwerveSetpointGenerator.Setpoint = SwerveSetpointGenerator.Setpoint(
        ChassisSpeeds(),
        Array(4){ SwerveModuleState() }
    )

    init{
        AutoBuilder.configureHolonomic(
            { poseEstimator.robotPose.inUnit(meters) },
            { poseEstimator.resetPose(it.ofUnit(meters)) },
            { currentSpeeds },
            { speeds ->
                velocityDrive(speeds, fieldRelative = false)
                log(ChassisSpeeds.struct, "PathPlanner/ChassisSpeeds", speeds)
            },
            HolonomicPathFollowerConfig(
                chassisConstants.robotTranslationPID.asPathPlannerConstants(),
                chassisConstants.robotRotationPID.asPathPlannerConstants(),
                moduleConstants.driveMotorMaxSpeed.siValue,
                kotlin.math.sqrt(chassisConstants.trackWidth.inUnit(meters).pow(2) + chassisConstants.wheelBase.inUnit(meters).pow(2)),
                chassisConstants.pathReplanningConfig
            ),
            {
                when (val alliance = DriverStation.getAlliance()){
                    Optional.empty<DriverStation.Alliance>() -> false

                    else -> (alliance.get() == DriverStation.Alliance.Red)
                }
            },
            this
        )
    }

    /* PUBLIC API */
    /**
     * The locations of all the modules with respect to the robot's center.
     */
    public val moduleTranslationsFromRobotCenter = listOf(
        UnitTranslation2d(chassisConstants.trackWidth/2,chassisConstants.wheelBase/2),
        UnitTranslation2d(chassisConstants.trackWidth/2,-chassisConstants.wheelBase/2),
        UnitTranslation2d(-chassisConstants.trackWidth/2,chassisConstants.wheelBase/2),
        UnitTranslation2d(-chassisConstants.trackWidth/2,-chassisConstants.wheelBase/2)
    )

    /**
     * The kinematics class for the drivetrain.
     */
    public val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(*moduleTranslationsFromRobotCenter.map{ it.inUnit(meters) }.toTypedArray())

    /**
     * The pose estimator of the [EncoderHolonomicDrivetrain].
     *
     * This can be changed to a different pose monitor if necessary.
     */
    public var poseEstimator: RobotPoseMonitor = SwervePoseMonitor(drivetrain = this, startingPose)

    /**
     * The current heading (the direction the robot is facing).
     * This is equivalent to the gyro heading if a gyro is specified; otherwise, the pose estimator
     * will calculate the robot's heading using encoders and odometry.
     *
     * If possible, specify a gyro, like a [frc.chargers.hardware.sensors.imu.ChargerNavX],
     * in order to get the best possible results.
     *
     * This value is automatically standardized to a 0 to 360 degree range; however,
     * the calculated heading
     *
     * Thus, it's more common to use this property to determine *change* in heading.
     * If the initial value of this property is stored, the amount of rotation since
     * that initial point can easily be determined by subtracting the initial heading
     * from the current heading.
     *
     * @see HeadingProvider
     */
    override val heading: Angle get() =
        (gyro?.heading ?: poseEstimator.heading).inputModulus(0.degrees..360.degrees)

    /**
     * A class that generates swerve setpoints for the drivetrain.
     */
    public val setpointGenerator: SwerveSetpointGenerator = SwerveSetpointGenerator(
        kinematics,
        moduleTranslationsFromRobotCenter.map{ it.inUnit(meters) }.toTypedArray()
    )

    /**
     * The distance the robot has traveled in total.
     */
    public val distanceTraveled: Distance
        get() = swerveModules.map{ it.wheelTravel }.average()

    /**
     * The current overall velocity of the robot.
     */
    public val velocity: Velocity
        get(){
            val speeds = currentSpeeds
            return hypot(speeds.xVelocity, speeds.yVelocity)
        }

    /**
     * The current [ChassisSpeeds] of the robot.
     */
    public val currentSpeeds: ChassisSpeeds
        get() = setpoint.chassisSpeeds

    /**
     * Fetches a [SwerveData] instance containing [SwerveModulePosition]s,
     * which store the distance traveled and the angle of each swerve module.
     */
    public val modulePositions: SwerveData<SwerveModulePosition>
        get() = swerveModules.map{ it.getModulePosition() }

    /**
     * Fetches a [SwerveData] instance containing [SwerveModulePosition]s,
     * which store the velocity and the angle of each swerve module.
     */
    public val moduleStates: SwerveData<SwerveModuleState>
        get() = swerveModules.map{ it.getModuleState() }

    /**
     * Fetches a [SwerveData] instance containing each module's drive angular velocity.
     */
    public val moduleAngularVelocities: SwerveData<AngularVelocity>
        get() = swerveModules.map{ it.driveAngularVelocity }

    /**
     * Fetches a [SwerveData] instance containing each module's drive linear velocity.
     */
    public val moduleLinearVelocities: SwerveData<Velocity>
        get() = swerveModules.map{ it.driveLinearVelocity }

    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    public val maxLinearVelocity: Velocity =
        abs(
            kinematics.toChassisSpeeds(
                *Array(4){ SwerveModuleState(moduleConstants.driveMotorMaxSpeed.siValue, Rotation2d(0.0)) } ,
            ).xVelocity
        )

    /**
     * The max angular velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential(with each being oriented at a 45 or -45 degrees angle),
     * then calculating the output using the kinematics object.
     */
    public val maxRotationalVelocity: AngularVelocity = abs(
        kinematics.toChassisSpeeds(
            SwerveModuleState(moduleConstants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(-45.0)),
            SwerveModuleState(-moduleConstants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(moduleConstants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(-moduleConstants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(-45.0))
        ).rotationSpeed
    )

    /**
     * Sets a rotation override for the drivetrain.
     */
    public fun setRotationOverride(rotationOverride: RotationOverride){
        this.rotationOverride = rotationOverride
    }

    /**
     * Removes a rotation override for the drivetrain.
     */
    public fun removeRotationOverride(){
        this.rotationOverride = null
    }

    /**
     * Creates a [SysIdRoutine] for characterizing a drivetrain's drive motors.
     */
    public fun getDriveSysIdRoutine(
        quasistaticRampRate: VoltageRate? = null,
        dynamicStepVoltage: Voltage? = null,
        timeout: Time? = null
    ): SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            quasistaticRampRate?.toWPI(), dynamicStepVoltage?.toWPI(), timeout?.toWPI(),
        ) { log("DriveSysIDState", it.toString()) },
        SysIdRoutine.Mechanism(
            { voltage ->
                setDriveVoltages(SwerveData.create{ voltage.toKmeasure() })
                setTurnDirections(SwerveData.create{ Angle(0.0) })
            },
            null, // no need for log consumer since data is recorded by advantagekit
            this
        )
    )

    /**
     * Creates a [SysIdRoutine] for characterizing a drivetrain's turn(azimuth) motors.
     */
    public fun getAzimuthSysIdRoutine(
        quasistaticRampRate: VoltageRate? = null,
        dynamicStepVoltage: Voltage? = null,
        timeout: Time? = null
    ): SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            quasistaticRampRate?.toWPI(), dynamicStepVoltage?.toWPI(), timeout?.toWPI(),
        ) { log("AzimuthSysIDState", it.toString()) },
        SysIdRoutine.Mechanism(
            { voltage ->
                setDriveVoltages(SwerveData.create{ Voltage(0.0) })
                setTurnVoltages(SwerveData.create{ voltage.toKmeasure() })
            },
            null, // no need for log consumer since data is recorded by advantagekit
            this
        )
    )


    /* Drive Functions */

    /**
     * A generic drive function; mainly used if driving at a specific velocity is not required, or during teleop.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun swerveDrive(
        xPower: Double,
        yPower: Double,
        rotationPower: Double,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ): Unit = swerveDrive(ChassisPowers(xPower,yPower,rotationPower), fieldRelative)

    /**
     * A generic drive function; mainly used if driving at a specific velocity is not required, or during teleop.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun swerveDrive(
        powers: ChassisPowers,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ){
        currentControlMode = ControlMode.OPEN_LOOP
        goal = ChassisSpeeds(
            powers.xPower * maxLinearVelocity.siValue,
            powers.yPower * maxLinearVelocity.siValue,
            powers.rotationPower * maxRotationalVelocity.siValue
        )
        log(ChassisSpeeds.struct, "ChassisSpeeds/GoalWithoutModifiers", goal)
        if (fieldRelative){
            goal = ChassisSpeeds.fromFieldRelativeSpeeds(goal, (heading + allianceFieldRelativeOffset).asRotation2d())
        }
    }

    /**
     * Drives the robot with specific speeds; uses closed loop control.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun velocityDrive(
        xVelocity: Velocity,
        yVelocity: Velocity,
        rotationVelocity: AngularVelocity,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ): Unit = velocityDrive(ChassisSpeeds(xVelocity, yVelocity, rotationVelocity), fieldRelative)

    /**
     * Drives the robot with specific speeds; uses closed loop control.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    public fun velocityDrive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ){
        currentControlMode = ControlMode.CLOSED_LOOP
        goal = if (fieldRelative){
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, (heading + allianceFieldRelativeOffset).asRotation2d())
        }else{
            speeds
        }
        log(ChassisSpeeds.struct, "GoalWithoutModifiers", goal)
    }

    /**
     * Sets driving voltages for each module.
     */
    public fun setDriveVoltages(voltages: SwerveData<Voltage>){
        currentControlMode = ControlMode.NONE

        swerveModules.zip(voltages).forEach{ (module, voltage) ->
            module.setDriveVoltage(voltage)
        }
    }

    /**
     * Sets turn voltages for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    public fun setTurnVoltages(voltages: SwerveData<Voltage>){
        currentControlMode = ControlMode.NONE

        swerveModules.zip(voltages).forEach{ (module, voltage) ->
            module.setTurnVoltage(voltage)
        }
    }

    /**
     * Sets azimuth directions for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    public fun setTurnDirections(directions: SwerveData<Angle>){
        currentControlMode = ControlMode.NONE

        swerveModules.zip(directions).forEach{ (module, direction) ->
            module.setDirection(direction)
        }
    }

    /**
     * Stops the drivetrain.
     */
    public fun stop(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        goal = ChassisSpeeds()
        setpoint = SwerveSetpointGenerator.Setpoint(
            ChassisSpeeds(),
            setpoint.moduleStates
        )
        swerveModules.forEach{
            it.setDriveVoltage(0.volts)
            it.setTurnVoltage(0.volts)
        }
    }

    /**
     * Stops the drivetrain in an X.
     */
    public fun stopInX(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        goal = ChassisSpeeds()
        setpoint = SwerveSetpointGenerator.Setpoint(
            ChassisSpeeds(),
            setpoint.moduleStates
        )
        setTurnDirections(
            SwerveData(topLeft = 45.degrees, topRight = (-45).degrees, bottomLeft = (-45).degrees, bottomRight = 45.degrees)
        )
        swerveModules.forEach{
            it.setDriveVoltage(0.volts)
        }
    }

    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        log("DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        log("OverallVelocityMetersPerSec", velocity.inUnit(meters / seconds))
        log(SwerveModuleState.struct, "DesiredModuleStates", setpoint.moduleStates.toList())
        log("HasRotationOverride", rotationOverride != null)
        log("RequestedControlMode", currentControlMode.toString())
        log(ChassisSpeeds.struct, "ChassisSpeeds/Setpoint", setpoint.chassisSpeeds)
        log(ChassisSpeeds.struct, "ChassisSpeeds/Goal", goal)
        log(ChassisSpeeds.struct, "ChassisSpeeds/Measured", currentSpeeds)

        if (DriverStation.isDisabled()) {
            stop()
            return
        }else if (currentControlMode == ControlMode.NONE){
            return
        }

        when (currentControlMode) {
            ControlMode.CLOSED_LOOP -> {
                val output = rotationOverride?.invoke(this)
                if (output != null) {
                    goal.omegaRadiansPerSecond = output.closedLoopRotation.siValue
                }
                goal = goal.discretize(driftRate = chassisConstants.closedLoopDiscretizationRate)
            }

            ControlMode.OPEN_LOOP -> {
                val output = rotationOverride?.invoke(this)
                if (output != null) {
                    goal.omegaRadiansPerSecond = output.openLoopRotation * maxRotationalVelocity.siValue
                }
                goal = goal.discretize(driftRate = chassisConstants.openLoopDiscretizationRate)
            }

            else -> {}
        }

        setpoint = setpointGenerator.generateSetpoint(
            constraints,
            setpoint,
            goal,
            ChargerRobot.LOOP_PERIOD.inUnit(seconds)
        )

        swerveModules.forEachIndexed { index, module ->
            when (currentControlMode){
                ControlMode.OPEN_LOOP -> module.setDesiredStateOpenLoop(setpoint.moduleStates[index])

                ControlMode.CLOSED_LOOP -> module.setDesiredStateClosedLoop(setpoint.moduleStates[index])

                else -> {}
            }
        }
    }
}