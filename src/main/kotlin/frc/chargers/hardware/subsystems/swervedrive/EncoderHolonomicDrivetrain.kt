@file:Suppress("unused", "MemberVisibilityCanBePrivate", "LeakingThis")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog.log
import frc.chargers.framework.tunable
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.HeadingProvider
import frc.chargers.hardware.sensors.imu.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.PoseEstimatingDrivetrain
import frc.chargers.utils.units.VoltageRate
import frc.chargers.utils.units.toKmeasure
import frc.chargers.utils.units.toWPI
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.Translation2d
import frc.chargers.wpilibextensions.angle
import frc.chargers.wpilibextensions.kinematics.*
import frc6328.SwerveSetpointGenerator
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.pow


/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 *
 * Note: TrackWidth is the horizontal length of the robot, while wheelBase is the vertical length of the robot.
 * Data from lists comes in Top Left, Top Right, bottom left, bottom right order.
 */
open class EncoderHolonomicDrivetrain(
    private val name: String,
    turnMotors: List<Motor>,
    // turn encoders are optional in sim
    turnEncoders: List<PositionEncoder?> = List(4){ null },
    driveMotors: List<Motor>,
    val constants: SwerveConstants,
    private val gyro: HeadingProvider? = null
): PoseEstimatingDrivetrain(), HeadingProvider {
    private val moduleNames = listOf("Modules/TopLeft", "Modules/TopRight", "Modules/BottomLeft", "Modules/BottomRight")
    private val swerveModules = List(4){ index ->
        SwerveModule(
            name = name + "/" + moduleNames[index],
            turnMotors[index],
            turnEncoders[index],
            driveMotors[index],
            constants
        )
    }
    private val moduleTranslationsFromRobotCenter = arrayOf(
        Translation2d(constants.trackWidth/2, constants.wheelBase/2),
        Translation2d(constants.trackWidth/2, -constants.wheelBase/2),
        Translation2d(-constants.trackWidth/2, constants.wheelBase/2),
        Translation2d(-constants.trackWidth/2, -constants.wheelBase/2)
    )
    private val kinematics = SwerveDriveKinematics(*moduleTranslationsFromRobotCenter) // A helper class that stores the characteristics of the drivetrain.
    private val constraints = SwerveSetpointGenerator.ModuleLimits(
        constants.driveMotorMaxSpeed.siValue,
        constants.driveMotorMaxAcceleration.siValue,
        constants.turnMotorMaxSpeed.siValue
    )
    private val setpointGenerator = SwerveSetpointGenerator(kinematics, moduleTranslationsFromRobotCenter) // Converts ChassisSpeeds to module states that respect velocity and acceleration constraints.
    private var goal = ChassisSpeeds() // The ultimate goal state of the drivetrain; with x, y and rotational velocities.
    // The current setpoint of the drivetrain;
    // which stores the target speeds and module states of the drivetrain.
    private var setpoint = SwerveSetpointGenerator.Setpoint(ChassisSpeeds(), Array(4){ SwerveModuleState() })
    // An enum class that stores the current control mode of the drivetrain.
    private enum class ControlMode{
        OPEN_LOOP,  // Represents open-loop control with no feedforward / PID
        CLOSED_LOOP, // Represents closed-loop control with feedforward & PID: this is far more accurate than open loop in terms of velocity
        NONE // Represents no control at all; this mode should be set when the drivetrain is not calling one of the drive functions.
    }
    private var currentControlMode = ControlMode.NONE
    private val robotWidget = ChargerRobot.FIELD.getObject(name)
    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(0.0),
        modulePositions.toTypedArray(),
        Pose2d()
    )
    private var calculatedHeading = Angle(0.0)
    private var previousWheelTravelDistances = MutableList(4){ Distance(0.0) }

    private fun updatePoseEstimation(){
        if (gyro != null){
            poseEstimator.update(
                Rotation2d(gyro.heading),
                modulePositions.toTypedArray()
            )
        }else{
            // wheelDeltas represent the difference in position moved during the loop,
            // as well as the current angle(not the change in angle).
            val wheelDeltas = modulePositions.mapIndexed{ i, originalPosition ->
                val delta = originalPosition.distanceMeters - previousWheelTravelDistances[i].inUnit(meters)
                previousWheelTravelDistances[i] = originalPosition.distanceMeters.ofUnit(meters)
                SwerveModulePosition(delta, originalPosition.angle)
            }
            calculatedHeading += kinematics.toTwist2d(*wheelDeltas.toTypedArray()).dtheta.ofUnit(radians)

            poseEstimator.update(
                Rotation2d(calculatedHeading),
                modulePositions.toTypedArray()
            )
        }
    }

    private val azimuthPID by tunable(constants.azimuthPID, "$name/azimuthPID")
        .onChange{ pid -> turnMotors.forEach{ it.configure(positionPID = pid) }  }
    private val velocityPID by tunable(constants.velocityPID, "$name/velocityPID")
        .onChange{ pid -> driveMotors.forEach{ it.configure(velocityPID = pid) } }

    init{
        log("RealGyroUsedInPoseEstimation", gyro != null)
        ChargerRobot.runPeriodicAtPeriod(
            constants.odometryUpdateRate,
            ::updatePoseEstimation
        )

        AutoBuilder.configureHolonomic(
            { robotPose },
            { resetPose(it) },
            { currentSpeeds },
            { speeds ->
                velocityDrive(speeds, fieldRelative = false)
                log("$name/PathPlanner/ChassisSpeeds", speeds)
            },
            HolonomicPathFollowerConfig(
                constants.robotTranslationPID,
                constants.robotRotationPID,
                constants.driveMotorMaxSpeed.siValue,
                kotlin.math.sqrt(constants.trackWidth.inUnit(meters).pow(2) + constants.wheelBase.inUnit(meters).pow(2)),
                constants.pathReplanningConfig
            ),
            { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }, // determines if alliance flip for paths is necessary
            this
        )
    }

    /* PUBLIC API */
    /**
     * The current robot pose of the drivetrain.
     */
    override val robotPose: Pose2d
        get() = poseEstimator.estimatedPosition

    /**
     * Resets the drivetrain's pose.
     */
    final override fun resetPose(pose: Pose2d) {
        calculatedHeading = pose.rotation.angle
        if (gyro is ZeroableHeadingProvider){
            gyro.zeroHeading(pose.rotation.angle)
            // here, we do not take the gyro heading directly.
            // If we do, we will still be reading the old gyro heading value,
            // as the new(zeroed) value will not be updated until the next loop.
            // In addition, the gyro will be zeroed to the pose's rotation next loop anyway
            poseEstimator.resetPosition(
                pose.rotation,
                modulePositions.toTypedArray(),
                pose
            )
        }else{
            poseEstimator.resetPosition(
                Rotation2d(gyro?.heading ?: calculatedHeading),
                modulePositions.toTypedArray(),
                pose
            )
        }
    }

    /**
     * Adds a vision measurement to the drivetrain.
     */
    override fun addVisionMeasurement(pose: Pose2d, timestamp: Time, stdDevs: Matrix<N3, N1>?) {
        // rotationSpeed is an extension property that converts
        // omegaRadiansPerSecond to a kmeasure AngularVelocity.
        if (currentSpeeds.rotationSpeed > 720.radians / 1.seconds){
            println("Gyro rotating too fast; vision measurements ignored.")
        }else{
            if (stdDevs != null){
                poseEstimator.addVisionMeasurement(pose, timestamp.inUnit(seconds), stdDevs)
            }else{
                poseEstimator.addVisionMeasurement(pose, timestamp.inUnit(seconds))
            }
        }
    }

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
        (gyro?.heading ?: calculatedHeading) % 360.degrees

    /**
     * The distance the robot has traveled in total.
     */
    val distanceTraveled: Distance
        get(){
            val currentPose = robotPose
            return hypot(currentPose.x.ofUnit(meters), currentPose.y.ofUnit(meters))
        }

    /**
     * The current overall velocity of the robot.
     */
    val velocity: Velocity
        get(){
            val speeds = currentSpeeds
            return hypot(speeds.xVelocity, speeds.yVelocity)
        }

    /**
     * The current [ChassisSpeeds] of the robot.
     */
    val currentSpeeds: ChassisSpeeds
        get() = setpoint.chassisSpeeds

    /**
     * Fetches a List containing [SwerveModulePosition]s,
     * which store the distance traveled and the angle of each swerve module.
     */
    val modulePositions get() = swerveModules.map{ it.getModulePosition() }

    /**
     * Fetches a List containing [SwerveModulePosition]s,
     * which store the velocity and the angle of each swerve module.
     */
    val moduleStates get() = swerveModules.map{ it.getModuleState() }

    /**
     * Fetches a List containing each module's drive angular velocity.
     */
    val moduleAngularVelocities get() = swerveModules.map{ it.driveAngularVelocity }

    /**
     * Fetches a List containing each module's drive linear velocity.
     */
    val moduleLinearVelocities get() = swerveModules.map{ it.driveLinearVelocity }

    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    val maxLinearVelocity: Velocity = constants.driveMotorMaxSpeed

    /**
     * The max angular velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential(with each being oriented at a 45 or -45 degrees angle),
     * then calculating the output using the kinematics object.
     */
    val maxRotationalVelocity: AngularVelocity = abs(
        kinematics.toChassisSpeeds(
            SwerveModuleState(constants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(-45.0)),
            SwerveModuleState(-constants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(constants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(-constants.driveMotorMaxSpeed.siValue, Rotation2d.fromDegrees(-45.0))
        ).omegaRadiansPerSecond
    ).ofUnit(radians / seconds)

    /**
     * Creates a [SysIdRoutine] for characterizing a drivetrain's drive motors.
     */
    fun getDriveSysIdRoutine(
        quasistaticRampRate: VoltageRate? = null,
        dynamicStepVoltage: Voltage? = null,
        timeout: Time? = null
    ): SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            quasistaticRampRate?.toWPI(), dynamicStepVoltage?.toWPI(), timeout?.toWPI(),
        ) { log("DriveSysIDState", it.toString()) },
        SysIdRoutine.Mechanism(
            { voltage ->
                setDriveVoltages(List(4){ voltage.toKmeasure() })
                setTurnDirections(List(4){ Angle(0.0) })
            },
            null, // no need for log consumer since data is recorded by logging
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
    fun swerveDrive(
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
    fun swerveDrive(
        powers: ChassisPowers,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ){
        currentControlMode = ControlMode.OPEN_LOOP
        goal = ChassisSpeeds(
            powers.xPower * maxLinearVelocity.siValue,
            powers.yPower * maxLinearVelocity.siValue,
            powers.rotationPower * maxRotationalVelocity.siValue
        )
        log("$name/ChassisSpeeds/GoalWithoutModifiers", goal)
        if (fieldRelative){
            val allianceFieldRelativeOffset = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                180.degrees
            }else{
                0.degrees
            }
            goal = ChassisSpeeds.fromFieldRelativeSpeeds(goal, Rotation2d(heading + allianceFieldRelativeOffset))
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
    fun velocityDrive(
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
    fun velocityDrive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean = RobotBase.isSimulation() || gyro != null
    ){
        currentControlMode = ControlMode.CLOSED_LOOP
        if (fieldRelative){
            val allianceFieldRelativeOffset = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                180.degrees
            }else{
                0.degrees
            }
            goal = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d(heading + allianceFieldRelativeOffset))
        }else{
            goal = speeds
        }
        log("$name/GoalWithoutModifiers", goal)
    }

    /**
     * Sets driving voltages for each module.
     */
    fun setDriveVoltages(voltages: List<Voltage>){
        currentControlMode = ControlMode.NONE
        swerveModules.zip(voltages).forEach{ (module, voltage) ->
            module.setDriveVoltage(voltage)
        }
    }

    /**
     * Sets turn voltages for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    fun setTurnVoltages(voltages: List<Voltage>){
        currentControlMode = ControlMode.NONE
        swerveModules.zip(voltages).forEach{ (module, voltage) ->
            module.setTurnVoltage(voltage)
        }
    }

    /**
     * Sets azimuth directions for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    fun setTurnDirections(directions: List<Angle>){
        currentControlMode = ControlMode.NONE
        swerveModules.zip(directions).forEach{ (module, direction) ->
            module.setDirection(direction)
        }
    }

    /**
     * Stops the drivetrain.
     */
    fun stop(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        goal = ChassisSpeeds()
        setpoint = SwerveSetpointGenerator.Setpoint(ChassisSpeeds(), setpoint.moduleStates)
        swerveModules.forEach{
            it.setDriveVoltage(0.volts)
            it.setTurnVoltage(0.volts)
        }
    }

    /**
     * Stops the drivetrain in an X.
     */
    fun stopInX(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        goal = ChassisSpeeds()
        setpoint = SwerveSetpointGenerator.Setpoint(ChassisSpeeds(), setpoint.moduleStates)
        setTurnDirections(listOf(45.degrees, -45.degrees, -45.degrees, 45.degrees))
        swerveModules.forEach{ it.setDriveVoltage(0.volts) }
    }

    /**
     * Re-syncs the turn relative encoders in case there is a discrepancy
     * between the turn relative encoders and the turn absolute encoders.
     */
    fun reSyncRelativeEncoders() {
        swerveModules.forEach{ it.syncTurnEncoder() }
    }

    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        log("$name/DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        log("$name/OverallVelocityMetersPerSec", velocity.inUnit(meters / seconds))
        log("$name/RequestedControlMode", currentControlMode)
        log("$name/DesiredModuleStates", setpoint.moduleStates.toList())
        log("$name/MeasuredModuleStates", moduleStates)
        log("$name/ChassisSpeeds/Setpoint", setpoint.chassisSpeeds)
        log("$name/ChassisSpeeds/Goal", goal)
        log("$name/ChassisSpeeds/Measured", currentSpeeds)
        log("$name/Pose2d", poseEstimator.estimatedPosition)
        robotWidget.pose = poseEstimator.estimatedPosition

        if (DriverStation.isDisabled()) {
            stop()
            return
        }else if (currentControlMode == ControlMode.NONE){
            return
        }

        goal = ChassisSpeeds.discretize(goal, 0.02)
        setpoint = setpointGenerator.generateSetpoint(
            constraints,
            setpoint,
            goal,
            0.02
        )
        swerveModules.forEachIndexed { index, module ->
            when (currentControlMode){
                ControlMode.OPEN_LOOP -> module.setDesiredStateOpenLoop(setpoint.moduleStates[index])

                ControlMode.CLOSED_LOOP -> module.setDesiredStateClosedLoop(setpoint.moduleStates[index])

                ControlMode.NONE -> {}
            }
        }
    }
}