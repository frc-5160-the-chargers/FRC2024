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
import frc.chargers.framework.tunable
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.HeadingProvider
import frc.chargers.hardware.sensors.imu.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.PoseEstimatingDrivetrain
import frc.chargers.utils.epsilonEquals
import frc.chargers.utils.units.VoltageRate
import frc.chargers.utils.units.toKmeasure
import frc.chargers.utils.units.toWPI
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.Translation2d
import frc.chargers.wpilibextensions.angle
import frc.chargers.wpilibextensions.kinematics.*
import monologue.Logged
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.pow

private fun ensureFour(type: String, list: List<*>) {
    require(list.size == 4){ "You must have four ${type}s." }
}
private var autoBuilderConfigured = false

/**
 * An implementation of Swerve drive, with encoders, to be used in future robot code.
 * Swerve drive is called four-wheel holonomic drive outside of FRC, hence the name.
 *
 * Note: TrackWidth is the horizontal length of the robot, while wheelBase is the vertical length of the robot.
 * Data from lists comes in Top Left, Top Right, bottom left, bottom right order.
 */
open class EncoderHolonomicDrivetrain(
    turnMotors: List<Motor>,
    // turn encoders are optional in sim
    turnEncoders: List<PositionEncoder?> = List(4){ null },
    driveMotors: List<Motor>,
    val constants: SwerveConstants,
    val gyro: HeadingProvider? = null
): PoseEstimatingDrivetrain() {
    init {
        ensureFour("turn motor", turnMotors)
        ensureFour("turn encoder", turnEncoders)
        ensureFour("drive motor", driveMotors)
    }
    // note: dont change this to list, as monologue can't traverse through lists
    private val swerveModules = Array(4){ index ->
        SwerveModule(
            turnMotors[index],
            turnEncoders[index],
            driveMotors[index],
            constants
        )
    }
    private val kinematics = SwerveDriveKinematics(
        Translation2d(constants.trackWidth/2, constants.wheelBase/2),
        Translation2d(constants.trackWidth/2, -constants.wheelBase/2),
        Translation2d(-constants.trackWidth/2, constants.wheelBase/2),
        Translation2d(-constants.trackWidth/2, -constants.wheelBase/2)
    ) // A helper class that stores the characteristics of the drivetrain.
    private var goal = ChassisSpeeds() // The ultimate goal state of the drivetrain; with x, y and rotational velocities.
    private var desiredStates = Array(4){ SwerveModuleState() } // the current desired module states(velocity + direction)
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
        this.modulePositions.toTypedArray(),
        Pose2d()
    )
    private var previousWheelTravelDistances = MutableList(4){ Distance(0.0) }
    var calculatedHeading = Angle(0.0)
        private set

    // allows for PID tuning.
    private val azimuthPID by tunable(constants.azimuthPID)
        .onChange{ pid -> turnMotors.forEach{ it.configure(positionPID = pid) }  }
    private val velocityPID by tunable(constants.velocityPID)
        .onChange{ pid -> driveMotors.forEach{ it.configure(velocityPID = pid) } }

    private val allianceFieldRelativeOffset get() =
        when (DriverStation.getAlliance().getOrNull()){
            DriverStation.Alliance.Red -> 180.degrees
            else -> 0.degrees
        }
    private val defaultFieldRelative = RobotBase.isSimulation() || gyro != null

    private fun updatePoseEstimation() {
        val currentPositions = this.modulePositions.toTypedArray()
        if (gyro != null){
            poseEstimator.update(Rotation2d(gyro.heading), currentPositions)
        }else{
            // wheelDeltas represent the difference in position moved during the loop,
            // as well as the current angle(not the change in angle).
            val wheelDeltas = currentPositions.mapIndexed{ i, originalPosition ->
                val delta = originalPosition.distanceMeters - previousWheelTravelDistances[i].inUnit(meters)
                previousWheelTravelDistances[i] = originalPosition.distanceMeters.ofUnit(meters)
                SwerveModulePosition(delta, originalPosition.angle)
            }
            calculatedHeading += kinematics.toTwist2d(*wheelDeltas.toTypedArray()).dtheta.ofUnit(radians)
            poseEstimator.update(Rotation2d(calculatedHeading), currentPositions)
        }
    }

    init{
        ChargerRobot.runPeriodicAtPeriod(
            constants.odometryUpdateRate,
            ::updatePoseEstimation
        )
        if (!autoBuilderConfigured) {
            AutoBuilder.configureHolonomic(
                { robotPose },
                { resetPose(it) },
                { currentSpeeds },
                { speeds ->
                    velocityDrive(speeds, fieldRelative = false)
                    log("PathPlanner/ChassisSpeeds", speeds)
                },
                HolonomicPathFollowerConfig(
                    constants.robotTranslationPID,
                    constants.robotRotationPID,
                    constants.driveMotorMaxSpeed.siValue,
                    kotlin.math.sqrt(constants.trackWidth.inUnit(meters).pow(2) + constants.wheelBase.inUnit(meters).pow(2)),
                    constants.pathReplanningConfig
                ),
                { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }, // determines if alliance flip for paths is necessary
                this // subsystem requirement
            )
            autoBuilderConfigured = true
        }
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
        if (currentSpeeds.rotationalVelocity > 720.radians / 1.seconds){
            println("Gyro rotating too fast; vision measurements ignored.")
        }else if (stdDevs != null){
            poseEstimator.addVisionMeasurement(pose, timestamp.inUnit(seconds), stdDevs)
        }else{
            poseEstimator.addVisionMeasurement(pose, timestamp.inUnit(seconds))
        }
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
    val currentSpeeds: ChassisSpeeds get() = kinematics.toChassisSpeeds(*moduleStates.toTypedArray())

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
    fun getSysIdRoutine(
        quasistaticRampRate: VoltageRate? = null,
        dynamicStepVoltage: Voltage? = null,
        timeout: Time? = null
    ): SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            quasistaticRampRate?.toWPI(), dynamicStepVoltage?.toWPI(), timeout?.toWPI(),
        ) { log("DriveSysIDState", it.toString()) },
        SysIdRoutine.Mechanism(
            { voltage ->
                val out = voltage.toKmeasure()
                setDriveVoltages(out, out, out, out)
                setTurnDirections(Angle(0.0), Angle(0.0), Angle(0.0), Angle(0.0))
            },
            null, // no need for log consumer since data is recorded by logging
            this
        )
    )

    /* Drive Functions */
    private fun setSpeeds(xVel: Double, yVel: Double, rotVel: Double, fieldRelative: Boolean) {
        goal = if (fieldRelative) {
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVel, yVel, rotVel,
                Rotation2d((gyro?.heading ?: calculatedHeading) + allianceFieldRelativeOffset)
            )
        }else{
            ChassisSpeeds(xVel, yVel, rotVel)
        }
    }

    /**
     * A generic drive function; mainly used if driving at a specific velocity is not required, or during teleop.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    fun swerveDrive(powers: ChassisPowers, fieldRelative: Boolean = defaultFieldRelative) =
        swerveDrive(powers.xPower, powers.yPower, powers.rotationPower, fieldRelative)

    /**
     * A generic drive function; mainly used if driving at a specific velocity is not required, or during teleop.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    fun swerveDrive(xPower: Double, yPower: Double, rotationPower: Double, fieldRelative: Boolean = defaultFieldRelative){
        if (xPower epsilonEquals 0.0 && yPower epsilonEquals 0.0 && rotationPower epsilonEquals 0.0) {
            this.stop()
            return
        }
        currentControlMode = ControlMode.OPEN_LOOP
        setSpeeds(
            xPower * maxLinearVelocity.siValue,
            yPower * maxLinearVelocity.siValue,
            rotationPower * maxRotationalVelocity.siValue,
            fieldRelative
        )
    }

    /**
     * Drives the robot with specific speeds; uses closed loop control.
     *
     * By default, will drive field-oriented in simulation(using calculated heading),
     * and in the real robot IF a gyro is provided.
     *
     * This value can be changed with the [fieldRelative] parameter.
     */
    fun velocityDrive(speeds: ChassisSpeeds, fieldRelative: Boolean = defaultFieldRelative) =
        velocityDrive(speeds.xVelocity, speeds.yVelocity, speeds.rotationalVelocity, fieldRelative)

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
        fieldRelative: Boolean = defaultFieldRelative
    ){
        currentControlMode = ControlMode.CLOSED_LOOP
        setSpeeds(xVelocity.siValue, yVelocity.siValue, rotationVelocity.siValue, fieldRelative)
    }

    /**
     * Sets driving voltages for each module.
     */
    fun setDriveVoltages(topLeft: Voltage, topRight: Voltage, bottomLeft: Voltage, bottomRight: Voltage){
        currentControlMode = ControlMode.NONE
        swerveModules[0].setDriveVoltage(topLeft)
        swerveModules[1].setDriveVoltage(topRight)
        swerveModules[2].setDriveVoltage(bottomLeft)
        swerveModules[3].setDriveVoltage(bottomRight)
    }

    /**
     * Sets turn voltages for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    fun setTurnVoltages(topLeft: Voltage, topRight: Voltage, bottomLeft: Voltage, bottomRight: Voltage){
        currentControlMode = ControlMode.NONE
        swerveModules[0].setTurnVoltage(topLeft)
        swerveModules[1].setTurnVoltage(topRight)
        swerveModules[2].setTurnVoltage(bottomLeft)
        swerveModules[3].setTurnVoltage(bottomRight)
    }

    /**
     * Sets azimuth directions for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    fun setTurnDirections(topLeft: Angle, topRight: Angle, bottomLeft: Angle, bottomRight: Angle){
        currentControlMode = ControlMode.NONE
        swerveModules[0].setDirection(topLeft)
        swerveModules[1].setDirection(topRight)
        swerveModules[2].setDirection(bottomLeft)
        swerveModules[3].setDirection(bottomRight)
    }

    /**
     * Stops the drivetrain.
     */
    fun stop(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        goal = ChassisSpeeds()
        swerveModules.forEach {
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
        setTurnDirections(45.degrees, -45.degrees, -45.degrees, 45.degrees)
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
        log("OverallVelocityMetersPerSec", velocity.inUnit(meters / seconds))
        log("RequestedControlMode", currentControlMode)
        log("ModuleStates/Desired", desiredStates)
        log("ModuleStates/Measured", moduleStates.toTypedArray())
        log("ChassisSpeeds/Desired", goal)
        log("ChassisSpeeds/Measured", currentSpeeds)
        log("Pose2d", poseEstimator.estimatedPosition)
        robotWidget.pose = poseEstimator.estimatedPosition

        if (DriverStation.isDisabled()) {
            stop()
            return
        }else if (currentControlMode == ControlMode.NONE){
            return
        }

        goal = ChassisSpeeds.discretize(goal, 0.02)
        desiredStates = kinematics.toSwerveModuleStates(goal)
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.driveMotorMaxSpeed.inUnit(meters / seconds))
        for (i in 0..<4) {
            val module = swerveModules[i]
            val desiredState = desiredStates[i]
            when (currentControlMode){
                ControlMode.OPEN_LOOP -> module.setDesiredStateOpenLoop(desiredState)
                ControlMode.CLOSED_LOOP -> module.setDesiredStateClosedLoop(desiredState)
                ControlMode.NONE -> {}
            }
        }
        swerveModules.forEachIndexed { index, module ->
            when (currentControlMode){
                ControlMode.OPEN_LOOP -> module.setDesiredStateOpenLoop(desiredStates[index])
                ControlMode.CLOSED_LOOP -> module.setDesiredStateClosedLoop(desiredStates[index])
                ControlMode.NONE -> {}
            }
        }
    }
}