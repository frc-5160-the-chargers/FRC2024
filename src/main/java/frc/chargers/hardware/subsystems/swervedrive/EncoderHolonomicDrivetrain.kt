@file:Suppress("RedundantVisibilityModifier", "unused", "MemberVisibilityCanBePrivate")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.motionprofiling.optimizeForContinuousInput
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.hardware.subsystems.robotposition.SwervePoseMonitor
import frc.chargers.hardware.subsystems.swervedrive.modulelowlevel.ModuleIO
import frc.chargers.hardware.subsystems.swervedrive.modulelowlevel.ModuleIOReal
import frc.chargers.hardware.subsystems.swervedrive.modulelowlevel.ModuleIOSim
import frc.chargers.pathplannerextensions.asPathPlannerConstants
import frc.chargers.utils.a
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.math.units.VoltageRate
import frc.chargers.utils.math.units.toKmeasure
import frc.chargers.utils.math.units.toWPI
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.chargers.wpilibextensions.kinematics.*
import frc.external.frc6328.SwerveSetpointGenerator
import org.littletonrobotics.junction.Logger.recordOutput
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
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
    public val logName: String = "Drivetrain(Swerve)",
    turnMotors: SwerveMotors<EncoderMotorController>,
    turnEncoders: SwerveEncoders<PositionEncoder>,
    driveMotors: SwerveMotors<EncoderMotorController>,
    private val hardwareData: SwerveHardwareData,
    private val controlData: SwerveControlData,
    private val useOnboardPID: Boolean = false,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(Distance(0.0), Distance(0.0), gyro?.heading ?: Angle(0.0)),
    poseSuppliers: List<VisionPoseSupplier> = listOf()
): SubsystemBase(), HeadingProvider {
    
    /* Private Implementation */
    private val wheelRadius = hardwareData.wheelDiameter / 2.0
    
    private fun createModuleIO(
        logInputs: LoggableInputsProvider,
        turnMotor: EncoderMotorController,
        turnEncoder: PositionEncoder,
        driveMotor: EncoderMotorController
    ): ModuleIO = if (RobotBase.isReal()){
        ModuleIOReal(
            logInputs, useOnboardPID, turnMotor,
            turnEncoder, driveMotor, hardwareData.turnGearRatio,
            hardwareData.driveGearRatio, hardwareData.couplingRatio
        )
    }else{
        ModuleIOSim(
            logInputs,
            turnMotorSim = DCMotorSim(
                hardwareData.turnMotorType,
                hardwareData.turnGearRatio,
                hardwareData.turnInertiaMoment.inUnit(kilo.grams * (meters * meters))
            ),
            driveMotorSim = DCMotorSim(
                hardwareData.driveMotorType,
                hardwareData.driveGearRatio,
                hardwareData.driveInertiaMoment.inUnit(kilo.grams * (meters * meters))
            )
        )
    }

    /**
     * An array of all the module IOs of the drivetrain.
     *
     * Order is always top left, top right, bottom left, bottom right.
     */
    private val moduleIOArray = a[
        createModuleIO(LoggableInputsProvider("$logName/TopLeftModule"), turnMotors.topLeft, turnEncoders.topLeft, driveMotors.topLeft),
        createModuleIO(LoggableInputsProvider("$logName/TopRightModule"), turnMotors.topRight, turnEncoders.topRight, driveMotors.topRight),
        createModuleIO(LoggableInputsProvider("$logName/BottomLeftModule"), turnMotors.bottomLeft, turnEncoders.bottomLeft, driveMotors.bottomLeft),
        createModuleIO(LoggableInputsProvider("$logName/BottomRightModule"), turnMotors.bottomRight, turnEncoders.bottomRight, driveMotors.bottomRight)
    ]

    /**
     * Stores motion profile setpoints for each module, in case it is utilized.
     */
    private val allModuleSetpoints: MutableMap<ModuleIO, AngularMotionProfileState> =
        mutableMapOf(
            *moduleIOArray.map{
                // creates pairs per module io
                it to AngularMotionProfileState()
            }.toTypedArray()
        )

    /**
     * Sets the direction of a module io,
     * filling out the appropriate pid constants
     * and applying motion profiling when necessary.
     */
    private fun setDirectionWithModifiers(moduleIO: ModuleIO, goal: Angle){
        when (controlData.azimuthControl){
            is SwerveAzimuthControl.ProfiledPID -> {
                var setpointState = allModuleSetpoints[moduleIO]!!
                val goalState = AngularMotionProfileState(goal)

                fun calculateSetpoint(): AngularMotionProfileState =
                    controlData.azimuthControl.motionProfile.calculate(
                        ChargerRobot.LOOP_PERIOD,
                        setpoint = setpointState,
                        goal = goalState,
                    )

                // optimizes profile states for continuous input
                optimizeForContinuousInput(
                    setpointState,
                    goalState,
                    moduleIO.direction,
                    continuousInputRange = 0.degrees..360.degrees
                )
                // calculates the new setpoint
                setpointState = calculateSetpoint()
                // setpoint already incremented; thus,
                // calculates the future motion profile state instead of the current one
                val futureSetpoint = calculateSetpoint()
                // refreshes the data storage for setpoints
                allModuleSetpoints[moduleIO] = setpointState

                // uses pid control to move to the calculated setpoint, to acheive the target goal.
                moduleIO.setDirectionSetpoint(
                    allModuleSetpoints[moduleIO]!!.position,
                    controlData.azimuthControl.pidConstants,
                    controlData.azimuthControl.ffEquation.calculatePlantInversion(
                        allModuleSetpoints[moduleIO]!!.velocity,
                        futureSetpoint.velocity
                    )
                )
            }

            is SwerveAzimuthControl.PID -> {
                moduleIO.setDirectionSetpoint(
                    goal,
                    controlData.azimuthControl.pidConstants
                )
            }
        }
    }

    /**
     * Sets the desired state of a singular module IO.
     */
    private fun setDesiredState(
        moduleIO: ModuleIO,
        state: SwerveModuleState,
        openLoop: Boolean
    ){
        // asAngle converts a Rotation2d to an Angle(kmeasure).
        val optimizedState = SwerveModuleState.optimize(
            state, moduleIO.direction.asRotation2d()
        )

        optimizedState.speedMetersPerSecond *= abs(
            (optimizedState.angle - moduleIO.direction.asRotation2d()).cos
        )

        setDirectionWithModifiers(moduleIO, optimizedState.angle.asAngle())

        if (openLoop){
            moduleIO.setDriveVoltage(
                optimizedState.speedMetersPerSecond
                    / hardwareData.maxModuleSpeed.inUnit(meters / seconds)
                    * 12.volts
            )
        }else{
            val speed = optimizedState.speedMetersPerSecond.ofUnit(meters / seconds) / wheelRadius
            moduleIO.setSpeedSetpoint(
                speed,
                controlData.velocityPID,
                controlData.velocityFF(speed) // returns a feedforward voltage
            )
        }
    }

    private val constraints = SwerveSetpointGenerator.ModuleLimits(
        hardwareData.maxModuleSpeed.siValue,
        hardwareData.maxModuleAcceleration.siValue,
        hardwareData.maxModuleRotationSpeed.siValue
    )

    private fun averageEncoderPosition() = moduleIOArray.map{ it.wheelTravel }.average()

    private val distanceOffset: Distance = averageEncoderPosition() * wheelRadius

    private var rotationOverride: RotationOverride? = null

    private enum class ControlMode{
        OPEN_LOOP,  // Represents open-loop control with no feedforward / PID
        CLOSED_LOOP, // Represents closed-loop control with feedforward & PID: this is far more accurate than open loop in terms of velocity
        NONE // Represents no control at all; this mode should be set when the drivetrain is not calling one of the drive functions.
    }

    private var currentControlMode: ControlMode = ControlMode.NONE

    private var goal: ChassisSpeeds = ChassisSpeeds()

    private var setpoint: SwerveSetpointGenerator.Setpoint = SwerveSetpointGenerator.Setpoint(
        ChassisSpeeds(),
        Array(4){ SwerveModuleState() }
    )


    private val allianceFieldRelativeOffset
        get() = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
            180.degrees
        }else{
            0.degrees
        }


    init{
        if (hardwareData.invertTurnMotors && RobotBase.isReal()){
            turnMotors.forEach{
                it.inverted = !it.inverted
            }
        }

        AutoBuilder.configureHolonomic(
            { poseEstimator.robotPose.inUnit(meters) },
            { poseEstimator.resetPose(it.ofUnit(meters)) },
            { currentSpeeds },
            { speeds ->
                velocityDrive(speeds, fieldRelative = false)
                recordOutput("$logName/pathplanningChassisSpeeds", speeds)
            },
            HolonomicPathFollowerConfig(
                controlData.robotTranslationPID.asPathPlannerConstants(),
                controlData.robotRotationPID.asPathPlannerConstants(),
                hardwareData.maxModuleSpeed.siValue,
                kotlin.math.sqrt(hardwareData.trackWidth.inUnit(meters).pow(2) + hardwareData.wheelBase.inUnit(meters).pow(2)),
                controlData.pathReplanConfig
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
        UnitTranslation2d(hardwareData.trackWidth/2,hardwareData.wheelBase/2),
        UnitTranslation2d(hardwareData.trackWidth/2,-hardwareData.wheelBase/2),
        UnitTranslation2d(-hardwareData.trackWidth/2,hardwareData.wheelBase/2),
        UnitTranslation2d(-hardwareData.trackWidth/2,-hardwareData.wheelBase/2)
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
    public var poseEstimator: RobotPoseMonitor = SwervePoseMonitor(
        drivetrain = this,
        poseSuppliers.toMutableList(),
        startingPose
    )

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
        get() = (averageEncoderPosition() * wheelRadius) - distanceOffset


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
     * Fetches an array of [SwerveModulePosition]s,
     * which store the distance traveled and the angle of each swerve module.
     *
     * Order is: TL, TR, BL, BR
     */
    public val modulePositions: List<SwerveModulePosition>
        get() = moduleIOArray.map{
            SwerveModulePosition(
                (it.wheelTravel * wheelRadius).inUnit(meters),
                it.direction.asRotation2d()
            )
        }

    /**
     * A list of each module's angular velocity
     */
    public val moduleAngularVelocities: List<AngularVelocity>
        get() = moduleIOArray.map{ it.speed }


    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    public val maxLinearVelocity: Velocity =
        abs(
            kinematics.toChassisSpeeds(
                *Array(4){ SwerveModuleState(hardwareData.maxModuleSpeed.siValue, Rotation2d(0.0)) } ,
            ).xVelocity
        )


    /**
     * The max angular velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential(with each being oriented at a 45 or -45 degrees angle),
     * then calculating the output using the kinematics object.
     */
    public val maxRotationalVelocity: AngularVelocity = abs(
        kinematics.toChassisSpeeds(
            SwerveModuleState(hardwareData.maxModuleSpeed.siValue, Rotation2d.fromDegrees(-45.0)),
            SwerveModuleState(-hardwareData.maxModuleSpeed.siValue, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(hardwareData.maxModuleSpeed.siValue, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(-hardwareData.maxModuleSpeed.siValue, Rotation2d.fromDegrees(-45.0))
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
        ) { recordOutput("$logName/DriveSysIDState", it.toString()) },
        SysIdRoutine.Mechanism(
            { voltage ->
                setDriveVoltages(List(4){ voltage.toKmeasure() })
                setTurnDirections(List(4){ Angle(0.0) })
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
        ) { recordOutput("$logName/AzimuthSysIDState", it.toString()) },
        SysIdRoutine.Mechanism(
            { voltage ->
                setDriveVoltages(List(4){ Voltage(0.0) })
                setTurnVoltages(List(4){ voltage.toKmeasure() })
            },
            null, // no need for log consumer since data is recorded by advantagekit
            this
        )
    )





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
        recordOutput("$logName/GoalWithoutModifiers", ChassisSpeeds.struct, goal)
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
        recordOutput("$logName/GoalWithoutModifiers", ChassisSpeeds.struct, goal)
    }


    /**
     * Sets driving voltages for each module.
     */
    public fun setDriveVoltages(voltages: List<Voltage>){
        currentControlMode = ControlMode.NONE
        require(voltages.size == 4){ "You must have 4 drive voltage parameters." }

        moduleIOArray.forEachIndexed{ index, moduleIO ->
            moduleIO.setDriveVoltage(voltages[index])
        }
    }


    /**
     * Sets turn voltages for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    public fun setTurnVoltages(voltages: List<Voltage>){
        currentControlMode = ControlMode.NONE
        require(voltages.size == 4){ "You must have 4 turn voltage parameters." }

        moduleIOArray.forEachIndexed{ index, moduleIO ->
            moduleIO.setTurnVoltage(voltages[index])
        }
    }


    /**
     * Sets azimuth directions for each module.
     * The standard order is top left, top right, bottom left, bottom right.
     */
    public fun setTurnDirections(directions: List<Angle>){
        currentControlMode = ControlMode.NONE
        require(directions.size == 4){ "You must have 4 turn direction parameters." }

        moduleIOArray.forEachIndexed{ index, moduleIO ->
            setDirectionWithModifiers(moduleIO, directions[index])
        }
    }


    /**
     * Stops the drivetrain.
     */
    public fun stop(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        moduleIOArray.forEach{
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
        setTurnDirections(
            listOf(45.degrees, (-45).degrees, (-45).degrees, 45.degrees)
        )
        moduleIOArray.forEach{
            it.setDriveVoltage(0.volts)
        }
    }


    /*
    public fun wheelRadiusCharacterizationCommand(clockwise: Boolean): Command = buildCommand {
        require(gyro != null){ "Gyro must be valid to characterize wheel radius" }
        addRequirements(this@EncoderHolonomicDrivetrain)

        var lastGyroYaw = Angle(0.0)
        var accumGyroAngle = Angle(0.0)

        val omegaLimiter = SlewRateLimiter(1.0)

        runOnce{
            omegaLimiter.reset(0.0)
        }


    }
     */


    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        recordOutput("$logName/DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        recordOutput("$logName/OverallVelocityMetersPerSec", velocity.inUnit(meters / seconds))
        recordOutput("$logName/DesiredModuleStates", *setpoint.moduleStates)
        recordOutput("$logName/ChassisSpeeds(Setpoint)", ChassisSpeeds.struct, setpoint.chassisSpeeds)
        recordOutput("$logName/ChassisSpeeds(Goal)", ChassisSpeeds.struct, goal)
        recordOutput("$logName/HasRotationOverride", rotationOverride != null)

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
                goal = goal.discretize(driftRate = controlData.closedLoopDiscretizationRate)
            }

            ControlMode.OPEN_LOOP -> {
                val output = rotationOverride?.invoke(this)
                if (output != null) {
                    goal.omegaRadiansPerSecond = output.openLoopRotation * maxRotationalVelocity.siValue
                }
                goal = goal.discretize(driftRate = controlData.openLoopDiscretizationRate)
            }

            else -> {}
        }

        setpoint = setpointGenerator.generateSetpoint(
            constraints,
            setpoint,
            goal,
            ChargerRobot.LOOP_PERIOD.inUnit(seconds)
        )


        moduleIOArray.forEachIndexed { index, moduleIO ->
            setDesiredState(
                moduleIO,
                setpoint.moduleStates[index],
                openLoop = currentControlMode == ControlMode.OPEN_LOOP
            )
        }
    }
}