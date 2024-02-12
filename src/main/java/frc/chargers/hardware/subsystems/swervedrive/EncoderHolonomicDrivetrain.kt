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
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.gyroscopes.*
import frc.chargers.hardware.subsystems.robotposition.SwervePoseMonitor
import frc.chargers.hardware.subsystems.swervedrive.module.*
import frc.chargers.hardware.subsystems.swervedrive.module.lowlevel.*
import frc.chargers.pathplannerextensions.asPathPlannerConstants
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
import org.littletonrobotics.junction.Logger.*
import java.util.Optional
import kotlin.math.pow


private val topLeftLogInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/TopLeftModule")

private val topRightLogInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/TopRightModule")

private val bottomLeftLogInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/BottomLeftModule")

private val bottomRightLogInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/BottomRightModule")

/**
 * A convenience function used to create an [EncoderHolonomicDrivetrain],
 * that automatically constructs real/sim versions depending on [RobotBase.isReal].
 */
public fun EncoderHolonomicDrivetrain(
    turnMotors: SwerveMotors<EncoderMotorController>,
    turnEncoders: SwerveEncoders<PositionEncoder> = turnMotors.getEncoders(),
    driveMotors: SwerveMotors<EncoderMotorController>,
    turnGearbox: DCMotor,
    driveGearbox: DCMotor,
    hardwareData: SwerveHardwareData,
    controlData: SwerveControlData,
    useOnboardPID: Boolean = false,
    gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    realPoseSuppliers: List<VisionPoseSupplier> = listOf(),
    simPoseSuppliers: List<VisionPoseSupplier> = listOf()
): EncoderHolonomicDrivetrain {
    if (RobotBase.isSimulation()){
        return EncoderHolonomicDrivetrain(
            topLeft = RioPIDSwerveModule(
                ModuleIOSim(
                    topLeftLogInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            topRight = RioPIDSwerveModule(
                ModuleIOSim(
                    topRightLogInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            bottomLeft = RioPIDSwerveModule(
                ModuleIOSim(
                    bottomLeftLogInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            bottomRight = RioPIDSwerveModule(
                ModuleIOSim(
                    bottomRightLogInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            hardwareData, controlData, gyro, startingPose, *simPoseSuppliers.toTypedArray()
        )
    }else{
        if (hardwareData.invertTurnMotors){
            turnMotors.apply{
                topLeft.inverted = !topLeft.inverted
                topRight.inverted = !topRight.inverted
                bottomLeft.inverted = !bottomLeft.inverted
                bottomRight.inverted = !bottomRight.inverted

                println("Top left inverted: " + topLeft.inverted)
                println("Top right inverted: " + topRight.inverted)
                println("bottom left inverted: " + bottomLeft.inverted)
                println("bottom right inverted: " + bottomRight.inverted)
            }
        }

        val topLeft: SwerveModule
        val topRight: SwerveModule
        val bottomLeft: SwerveModule
        val bottomRight: SwerveModule


        if (useOnboardPID){
            // SmartEncoderMotorController is a variant of EncoderMotorController
            // that allows for closed loop control
            require(turnMotors.containsMotors<SmartEncoderMotorController>()){
                "the turning motors of the drivetrain do not support onboard PID control; however, onboard PID control was requested."
            }

            require(driveMotors.containsMotors<SmartEncoderMotorController>()){
                "the driving motors of the drivetrain do not support onboard PID control; however, onboard PID control was requested."
            }

            // casts are safe, as code will error
            // if motors are not SmartEncoderMotorControllers

            @Suppress("Unchecked_Cast")
            turnMotors as SwerveMotors<SmartEncoderMotorController>

            @Suppress("Unchecked_Cast")
            driveMotors as SwerveMotors<SmartEncoderMotorController>


            topLeft = OnboardPIDSwerveModule(
                topLeftLogInputs,
                controlData,
                turnMotors.topLeft,
                turnEncoders.topLeft,
                driveMotors.topLeft,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

            topRight = OnboardPIDSwerveModule(
                topRightLogInputs,
                controlData,
                turnMotors.topRight,
                turnEncoders.topRight,
                driveMotors.topRight,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

            bottomLeft = OnboardPIDSwerveModule(
                bottomLeftLogInputs,
                controlData,
                turnMotors.bottomLeft,
                turnEncoders.bottomLeft,
                driveMotors.bottomLeft,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

            bottomRight = OnboardPIDSwerveModule(
                bottomRightLogInputs,
                controlData,
                turnMotors.bottomRight,
                turnEncoders.bottomRight,
                driveMotors.bottomRight,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

        }else{
            topLeft = RioPIDSwerveModule(
                ModuleIOReal(
                    topLeftLogInputs,
                    turnMotor = turnMotors.topLeft,
                    turnEncoder = turnEncoders.topLeft,
                    driveMotor = driveMotors.topLeft,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )

            topRight = RioPIDSwerveModule(
                ModuleIOReal(
                    topRightLogInputs,
                    turnMotor = turnMotors.topRight,
                    turnEncoder = turnEncoders.topRight,
                    driveMotor = driveMotors.topRight,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )

            bottomLeft = RioPIDSwerveModule(
                ModuleIOReal(
                    bottomLeftLogInputs,
                    turnMotor = turnMotors.bottomLeft,
                    turnEncoder = turnEncoders.bottomLeft,
                    driveMotor = driveMotors.bottomLeft,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )

            bottomRight = RioPIDSwerveModule(
                ModuleIOReal(
                    bottomRightLogInputs,
                    turnMotor = turnMotors.bottomRight,
                    turnEncoder = turnEncoders.bottomRight,
                    driveMotor = driveMotors.bottomRight,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )
        }

        return EncoderHolonomicDrivetrain(
            topLeft, topRight, bottomLeft, bottomRight,
            hardwareData, controlData, gyro, startingPose, *realPoseSuppliers.toTypedArray()
        )
    }
}








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
    private val topLeft: SwerveModule,
    private val topRight: SwerveModule,
    private val bottomLeft: SwerveModule,
    private val bottomRight: SwerveModule,
    public val hardwareData: SwerveHardwareData,
    public val controlData: SwerveControlData,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
): SubsystemBase(), HeadingProvider {
    /* Private Implementation */
    private val wheelRadius = hardwareData.wheelDiameter / 2.0

    private val moduleArray = arrayOf(topLeft,topRight,bottomLeft,bottomRight)

    private val moduleLocations = listOf(
        UnitTranslation2d(hardwareData.trackWidth/2,hardwareData.wheelBase/2),
        UnitTranslation2d(hardwareData.trackWidth/2,-hardwareData.wheelBase/2),
        UnitTranslation2d(-hardwareData.trackWidth/2,hardwareData.wheelBase/2),
        UnitTranslation2d(-hardwareData.trackWidth/2,-hardwareData.wheelBase/2)
    )

    private val constraints = SwerveSetpointGenerator.ModuleLimits(
        hardwareData.maxModuleSpeed.siValue,
        hardwareData.maxModuleAcceleration.siValue,
        hardwareData.maxModuleRotationSpeed.siValue
    )

    private fun averageEncoderPosition() = moduleArray.map{it.wheelTravel}.average()

    private val distanceOffset: Distance = averageEncoderPosition() * wheelRadius

    private var rotationOverride: RotationOverride = { null }

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


    init{
        AutoBuilder.configureHolonomic(
            { poseEstimator.robotPose.inUnit(meters) },
            { poseEstimator.resetPose(it.ofUnit(meters)) },
            { currentSpeeds },
            { speeds ->
                velocityDrive(speeds, fieldRelative = false)
                recordOutput("Drivetrain(Swerve)/pathplanningChassisSpeeds", speeds)
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
     * The pose estimator of the [EncoderHolonomicDrivetrain].
     *
     * This can be changed to a different pose monitor if necessary.
     */
    public var poseEstimator: RobotPoseMonitor = SwervePoseMonitor(
        drivetrain = this,
        startingPose = startingPose,
        *poseSuppliers,
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
     * The kinematics class for the drivetrain.
     */
    public val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*moduleLocations.map{ it.inUnit(meters) }.toTypedArray())


    /**
     * A class that generates swerve setpoints for the drivetrain.
     */
    public val setpointGenerator: SwerveSetpointGenerator = SwerveSetpointGenerator(
        kinematics,
        moduleLocations.map{ it.inUnit(meters) }.toTypedArray()
    )


    /**
     * The distance the robot has traveled in total.
     */
    public val distanceTraveled: Distance get() =
        (averageEncoderPosition() * wheelRadius) - distanceOffset


    /**
     * The current overall velocity of the robot.
     */
    public val velocity: Velocity get(){
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
        get() = listOf(
            topLeft.getModulePosition(wheelRadius),
            topRight.getModulePosition(wheelRadius),
            bottomLeft.getModulePosition(wheelRadius),
            bottomRight.getModulePosition(wheelRadius)
        )

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
        this.rotationOverride = { null }
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
        ) { recordOutput("Drivetrain(Swerve)/DriveSysIDState", it.toString()) },
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
        ) { recordOutput("Drivetrain(Swerve)/AzimuthSysIDState", it.toString()) },
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
        recordOutput("Drivetrain(Swerve)/BareGoal", ChassisSpeeds.struct, goal)
        if (fieldRelative){
            goal = ChassisSpeeds.fromFieldRelativeSpeeds(goal, heading.asRotation2d())
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
    ): Unit = velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationVelocity), fieldRelative)


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
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading.asRotation2d())
        }else{
            speeds
        }
        recordOutput("Drivetrain(Swerve)/BareGoal", ChassisSpeeds.struct, goal)
    }


    /**
     * Sets driving voltages for each module.
     */
    public fun setDriveVoltages(voltages: List<Voltage>){
        currentControlMode = ControlMode.NONE
        require(voltages.size == 4){ "You must have 4 drive voltage parameters." }

        topLeft.driveVoltage = voltages[0]
        topRight.driveVoltage = voltages[1]
        bottomLeft.driveVoltage = voltages[2]
        bottomRight.driveVoltage = voltages[3]
    }



    public fun setTurnVoltages(voltages: List<Voltage>){
        currentControlMode = ControlMode.NONE
        require(voltages.size == 4){ "You must have 4 turn voltage parameters." }

        topLeft.turnVoltage = voltages[0]
        topRight.turnVoltage = voltages[1]
        bottomLeft.turnVoltage = voltages[2]
        bottomRight.turnVoltage = voltages[3]
    }


    public fun setTurnDirections(directions: List<Angle>){
        currentControlMode = ControlMode.NONE
        require(directions.size == 4){ "You must have 4 turn direction parameters." }

        topLeft.setDirection(directions[0])
        topRight.setDirection(directions[1])
        bottomLeft.setDirection(directions[2])
        bottomRight.setDirection(directions[3])
    }


    /**
     * Stops the drivetrain.
     */
    public fun stop(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        topLeft.halt()
        topRight.halt()
        bottomLeft.halt()
        bottomRight.halt()
    }

    /**
     * Stops the drivetrain in an X.
     */
    public fun stopInX(){
        // prevents driving anywhere else
        currentControlMode = ControlMode.NONE
        topLeft.setDirectionalPower(0.0,45.degrees)
        topRight.setDirectionalPower(0.0, (-45).degrees)
        bottomLeft.setDirectionalPower(0.0, (-45).degrees)
        bottomRight.setDirectionalPower(0.0,45.degrees)
    }


    /**
     * Stalls the drivetrain motors in the forward direction, using the kS constant provided in the control scheme.
     */
    public fun stallForwards(){
        if (abs(controlData.velocityFF.kS) > 0.05.volts){
            setDriveVoltages(List(4){ controlData.velocityFF.kS - 0.05.volts })
        }else{
            setDriveVoltages(List(4){ controlData.velocityFF.kS })
        }
    }


    /**
     * Stalls the drivetrain motors in the backward direction, using the kS constant provided in the control scheme.
     */
    public fun stallBackwards(){
        if (abs(controlData.velocityFF.kS) > 0.05.volts){
            setDriveVoltages(List(4){ -controlData.velocityFF.kS + 0.05.volts })
        }else{
            setDriveVoltages(List(4){ -controlData.velocityFF.kS })
        }
    }


    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        recordOutput("Drivetrain(Swerve)/DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        recordOutput("Drivetrain(Swerve)/OverallVelocityMetersPerSec", velocity.inUnit(meters / seconds))

        if (DriverStation.isDisabled()) {
            stop()
            return
        }else if (currentControlMode == ControlMode.NONE){
            return
        }

        when (currentControlMode) {
            ControlMode.CLOSED_LOOP -> {
                val output = rotationOverride(this)
                if (output != null) {
                    goal.omegaRadiansPerSecond = output.closedLoopRotation.siValue
                }
                goal = goal.discretize(driftRate = controlData.closedLoopDiscretizationRate)
            }

            ControlMode.OPEN_LOOP -> {
                val output = rotationOverride(this)
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

        recordOutput("Drivetrain(Swerve)/rawKinematicsStates", SwerveModuleState.struct, *kinematics.toSwerveModuleStates(goal))
        recordOutput("Drivetrain(Swerve)/DesiredModuleStates", SwerveModuleState.struct, *setpoint.moduleStates)
        recordOutput("Drivetrain(Swerve)/ChassisSpeeds(Setpoint)", ChassisSpeeds.struct, setpoint.chassisSpeeds)
        recordOutput("Drivetrain(Swerve)/ChassisSpeeds(Goal)", ChassisSpeeds.struct, goal)

        when (currentControlMode) {
            ControlMode.CLOSED_LOOP -> {
                moduleArray.forEachIndexed { i, module ->
                    module.setDirectionalVelocity(
                        setpoint.moduleStates[i].speedMetersPerSecond.ofUnit(meters / seconds) / wheelRadius,
                        setpoint.moduleStates[i].angle.asAngle()
                    )
                }
            }

            ControlMode.OPEN_LOOP -> {
                moduleArray.forEachIndexed { i, module ->
                    module.setDirectionalPower(
                        setpoint.moduleStates[i].speedMetersPerSecond / hardwareData.maxModuleSpeed.siValue,
                        setpoint.moduleStates[i].angle.asAngle()
                    )
                }
            }

            else -> {}
        }

    }
}