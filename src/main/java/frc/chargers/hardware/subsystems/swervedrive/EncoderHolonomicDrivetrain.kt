@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.SmartEncoderMotorController
import frc.chargers.hardware.sensors.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.gyroscopes.*
import frc.chargers.hardware.subsystems.differentialdrive.DifferentialDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.module.*
import frc.chargers.hardware.subsystems.swervedrive.module.lowlevel.*
import frc.chargers.pathplannerextensions.asPathPlannerConstants
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.chargers.wpilibextensions.kinematics.*
import org.littletonrobotics.junction.Logger.*
import java.util.Optional
import kotlin.math.pow


private val topLeftModInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/TopLeftModule")

private val topRightModInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/TopRightModule")

private val bottomLeftModInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/BottomLeftModule")

private val bottomRightModInputs: LoggableInputsProvider = LoggableInputsProvider("Drivetrain(Swerve)/BottomRightModule")

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
                    topLeftModInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            topRight = RioPIDSwerveModule(
                ModuleIOSim(
                    topRightModInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            bottomLeft = RioPIDSwerveModule(
                ModuleIOSim(
                    bottomLeftModInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            bottomRight = RioPIDSwerveModule(
                ModuleIOSim(
                    bottomRightModInputs,
                    turnGearbox, driveGearbox, hardwareData.turnGearRatio, hardwareData.driveGearRatio, hardwareData.turnInertiaMoment, hardwareData.driveInertiaMoment
                ), controlData
            ),
            hardwareData, controlData, gyro, startingPose, *simPoseSuppliers.toTypedArray()
        )
    }else{
        if (hardwareData.invertTurnMotors){
            turnMotors.apply{
                println(topLeft.inverted)
                println(topRight.inverted)
                println(bottomLeft.inverted)
                println(bottomRight.inverted)
                topLeft.inverted = !topLeft.inverted
                topRight.inverted = !topRight.inverted
                bottomLeft.inverted = !bottomLeft.inverted
                bottomRight.inverted = !bottomRight.inverted
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
                topLeftModInputs,
                controlData,
                turnMotors.topLeft,
                turnEncoders.topLeft,
                driveMotors.topLeft,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

            topRight = OnboardPIDSwerveModule(
                topRightModInputs,
                controlData,
                turnMotors.topRight,
                turnEncoders.topRight,
                driveMotors.topRight,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

            bottomLeft = OnboardPIDSwerveModule(
                bottomLeftModInputs,
                controlData,
                turnMotors.bottomLeft,
                turnEncoders.bottomLeft,
                driveMotors.bottomLeft,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

            bottomRight = OnboardPIDSwerveModule(
                bottomRightModInputs,
                controlData,
                turnMotors.bottomRight,
                turnEncoders.bottomRight,
                driveMotors.bottomRight,
                hardwareData.driveGearRatio, hardwareData.turnGearRatio
            )

        }else{
            topLeft = RioPIDSwerveModule(
                ModuleIOReal(
                    topLeftModInputs,
                    turnMotor = turnMotors.topLeft,
                    turnEncoder = turnEncoders.topLeft,
                    driveMotor = driveMotors.topLeft,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )

            topRight = RioPIDSwerveModule(
                ModuleIOReal(
                    topRightModInputs,
                    turnMotor = turnMotors.topRight,
                    turnEncoder = turnEncoders.topRight,
                    driveMotor = driveMotors.topRight,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )

            bottomLeft = RioPIDSwerveModule(
                ModuleIOReal(
                    bottomLeftModInputs,
                    turnMotor = turnMotors.bottomLeft,
                    turnEncoder = turnEncoders.bottomLeft,
                    driveMotor = driveMotors.bottomLeft,
                    hardwareData.driveGearRatio, hardwareData.turnGearRatio
                ),
                controlData
            )

            bottomRight = RioPIDSwerveModule(
                ModuleIOReal(
                    bottomRightModInputs,
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
    public val topLeft: SwerveModule,
    public val topRight: SwerveModule,
    public val bottomLeft: SwerveModule,
    public val bottomRight: SwerveModule,
    public val hardwareData: SwerveHardwareData,
    public val controlData: SwerveControlData,
    public val gyro: HeadingProvider? = null,
    startingPose: UnitPose2d = UnitPose2d(),
    vararg poseSuppliers: VisionPoseSupplier,
): SubsystemBase(), HeadingProvider, DifferentialDrivetrain {
    /* Private Implementation */
    private val wheelRadius = hardwareData.wheelDiameter / 2.0
    private val moduleArray = arrayOf(topLeft,topRight,bottomLeft,bottomRight)
    private fun averageEncoderPosition() = moduleArray.map{it.wheelTravel}.average()

    private val distanceOffset: Distance = averageEncoderPosition() * wheelRadius

    /*
    OPEN_LOOP indicates percent-out(power) based drive,
    while CLOSED_LOOP uses PID and feedforward for velocity-based driving.
     */
    private enum class ControlMode{
        OPEN_LOOP,CLOSED_LOOP
    }
    private var currentControlMode: ControlMode = ControlMode.CLOSED_LOOP

    init{
        AutoBuilder.configureHolonomic(
            { poseEstimator.robotPose.inUnit(meters) },
            { poseEstimator.resetPose(it.ofUnit(meters)) },
            { currentSpeeds },
            { speeds -> velocityDrive(speeds, fieldRelative = false) },
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

                    else -> alliance.get() == DriverStation.Alliance.Red
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
    override val heading: Angle get() = poseEstimator.heading.inputModulus(0.degrees..360.degrees)


    /**
     * The kinematics class for the drivetrain.
     */
    public val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        UnitTranslation2d(hardwareData.trackWidth/2,hardwareData.wheelBase/2).inUnit(meters),
        UnitTranslation2d(hardwareData.trackWidth/2,-hardwareData.wheelBase/2).inUnit(meters),
        UnitTranslation2d(-hardwareData.trackWidth/2,hardwareData.wheelBase/2).inUnit(meters),
        UnitTranslation2d(-hardwareData.trackWidth/2,-hardwareData.wheelBase/2).inUnit(meters)
    )


    /**g
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
        get() = kinematics.toChassisSpeeds(*currentModuleStates.toArray())


    /**
     * A getter/setter variable that can either fetch the current speeds
     * of each swerve module,
     * or can set the desired speeds of each swerve module.
     * It is recommended that the [swerveDrive] and [velocityDrive] functions are used instead.
     */
    public var currentModuleStates: ModuleStateGroup
        get() = ModuleStateGroup(
            topLeftState = topLeft.getModuleState(wheelRadius),
            topRightState = topRight.getModuleState(wheelRadius),
            bottomLeftState = bottomLeft.getModuleState(wheelRadius),
            bottomRightState = bottomRight.getModuleState(wheelRadius)
        ).also{
            recordOutput("Drivetrain(Swerve)/CurrentModuleStates", SwerveModuleState.struct, it.topLeftState, it.topRightState, it.bottomLeftState, it.bottomRightState)
        }
        set(ms){
            ms.desaturate(hardwareData.maxModuleSpeed)
            if (currentControlMode == ControlMode.CLOSED_LOOP){
                topLeft.setDirectionalVelocity(ms.topLeftSpeed / wheelRadius,ms.topLeftAngle)
                topRight.setDirectionalVelocity(ms.topRightSpeed / wheelRadius,ms.topRightAngle)
                bottomLeft.setDirectionalVelocity(ms.bottomLeftSpeed / wheelRadius,ms.bottomLeftAngle)
                bottomRight.setDirectionalVelocity(ms.bottomRightSpeed / wheelRadius,ms.bottomRightAngle)
            }else{
                topLeft.setDirectionalPower((ms.topLeftSpeed/hardwareData.maxModuleSpeed).siValue, ms.topLeftAngle)
                topRight.setDirectionalPower((ms.topRightSpeed/hardwareData.maxModuleSpeed).siValue, ms.topRightAngle)
                bottomLeft.setDirectionalPower((ms.bottomLeftSpeed/hardwareData.maxModuleSpeed).siValue, ms.bottomLeftAngle)
                bottomRight.setDirectionalPower((ms.bottomRightSpeed/hardwareData.maxModuleSpeed).siValue, ms.bottomRightAngle)
            }
            recordOutput("Drivetrain(Swerve)/DesiredModuleStates", SwerveModuleState.struct, ms.topLeftState,ms.topRightState,ms.bottomLeftState,ms.bottomRightState)
        }


    /**
     * Gets the current module positions of each swerve module.
     * Returns a [ModulePositionGroup] object,
     * a wrapper around 4 SwerveModulePosition objects with units support.
     */
    public val currentModulePositions: ModulePositionGroup
        get() = ModulePositionGroup(
            topLeftDistance = topLeft.wheelTravel.inUnit(radians) * wheelRadius,
            topRightDistance = topRight.wheelTravel.inUnit(radians) * wheelRadius,
            bottomLeftDistance = bottomLeft.wheelTravel.inUnit(radians) * wheelRadius,
            bottomRightDistance = bottomRight.wheelTravel.inUnit(radians) * wheelRadius,
            topLeftAngle = topLeft.direction,
            topRightAngle = topRight.direction,
            bottomLeftAngle = bottomLeft.direction,
            bottomRightAngle = bottomRight.direction
        )


    /**
     * The max linear velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential,
     * then calculating the output using the kinematics object.
     */
    public val maxLinearVelocity: Velocity = abs(kinematics.toChassisSpeeds(
        *ModuleStateGroup(
            topLeftSpeed = hardwareData.maxModuleSpeed,
            topRightSpeed = hardwareData.maxModuleSpeed,
            bottomLeftSpeed = hardwareData.maxModuleSpeed,
            bottomRightSpeed = hardwareData.maxModuleSpeed,
            topLeftAngle = Angle(0.0),
            topRightAngle = Angle(0.0),
            bottomLeftAngle = Angle(0.0),
            bottomRightAngle = Angle(0.0)
        ).toArray()
    ).xVelocity)


    /**
     * The max angular velocity of the drivetrain, calculated by simulating
     * driving each swerve module at their maximum potential(with each being oriented at a 45 or -45 degrees angle),
     * then calculating the output using the kinematics object.
     */
    public val maxRotationalVelocity: AngularVelocity = abs(kinematics.toChassisSpeeds(
        *ModuleStateGroup(
            topLeftSpeed = hardwareData.maxModuleSpeed,
            topRightSpeed = -hardwareData.maxModuleSpeed,
            bottomLeftSpeed = hardwareData.maxModuleSpeed,
            bottomRightSpeed = -hardwareData.maxModuleSpeed,
            topLeftAngle = -45.degrees,
            topRightAngle = 45.degrees,
            bottomLeftAngle = 45.degrees,
            bottomRightAngle = -45.degrees
        ).toArray()
    ).rotationSpeed)


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
        if (DriverStation.isDisabled() || (powers.xPower == 0.0 && powers.yPower == 0.0 && powers.rotationPower == 0.0)){
            topLeft.halt()
            topRight.halt()
            bottomLeft.halt()
            bottomRight.halt()
            return
        }
        currentControlMode = ControlMode.OPEN_LOOP
        var speeds = powers.toChassisSpeeds(maxLinearVelocity,maxRotationalVelocity)
        if (fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading.asRotation2d())
        // extension function defined in chargerlib
        speeds = speeds.discretize(driftRate = controlData.openLoopDiscretizationRate)
        val stateArray = kinematics.toSwerveModuleStates(speeds)

        currentModuleStates = ModuleStateGroup(
            topLeftState = stateArray[0],
            topRightState = stateArray[1],
            bottomLeftState = stateArray[2],
            bottomRightState = stateArray[3]
        )

        currentControlMode = ControlMode.CLOSED_LOOP
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
        if (DriverStation.isDisabled()){
            topLeft.halt()
            topRight.halt()
            bottomLeft.halt()
            bottomRight.halt()
            return
        }
        currentControlMode = ControlMode.CLOSED_LOOP
        var newSpeeds: ChassisSpeeds =
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(speeds,heading.asRotation2d()) else speeds
        // extension function defined in chargerlib
        newSpeeds = newSpeeds.discretize(driftRate = controlData.closedLoopDiscretizationRate)
        val stateArray = kinematics.toSwerveModuleStates(newSpeeds)
        currentModuleStates = ModuleStateGroup(
            topLeftState = stateArray[0],
            topRightState = stateArray[1],
            bottomLeftState = stateArray[2],
            bottomRightState = stateArray[3]
        )
        currentControlMode = ControlMode.CLOSED_LOOP
    }


    override fun tankDrive(leftPower: Double, rightPower: Double) {
        topLeft.setDirectionalPower(leftPower,0.0.degrees)
        bottomLeft.setDirectionalPower(leftPower,0.0.degrees)
        topRight.setDirectionalPower(rightPower,0.0.degrees)
        bottomRight.setDirectionalPower(rightPower,0.0.degrees)
    }


    /**
     * Stops the drivetrain.
     */
    override fun stop(){
        topLeft.halt()
        topRight.halt()
        bottomLeft.halt()
        bottomRight.halt()
    }

    /**
     * Stops the drivetrain in an X.
     */
    public fun stopInX(){
        topLeft.setDirectionalPower(0.0,45.degrees)
        topRight.setDirectionalPower(0.0,-45.degrees)
        bottomLeft.setDirectionalPower(0.0,-45.degrees)
        bottomRight.setDirectionalPower(0.0,45.degrees)
    }


    /**
     * Stalls the drivetrain motors in the forward direction, using the kS constant provided in the control scheme.
     */
    public fun stallForwards(){
        // subtracts 0.05 volts from the stall voltage to correct for inaccuracies with kS constant.
        topLeft.driveVoltage = controlData.velocityFF.kS - 0.05.volts
        topRight.driveVoltage = controlData.velocityFF.kS - 0.05.volts
        bottomLeft.driveVoltage = controlData.velocityFF.kS - 0.05.volts
        bottomRight.driveVoltage = controlData.velocityFF.kS - 0.05.volts
    }


    /**
     * Stalls the drivetrain motors in the backward direction, using the kS constant provided in the control scheme.
     */
    public fun stallBackwards(){
        // adds 0.05 volts from the stall voltage to correct for inaccuracies with kS constant.
        topLeft.driveVoltage = -controlData.velocityFF.kS + 0.05.volts
        topRight.driveVoltage = -controlData.velocityFF.kS + 0.05.volts
        bottomLeft.driveVoltage = -controlData.velocityFF.kS + 0.05.volts
        bottomRight.driveVoltage = -controlData.velocityFF.kS + 0.05.volts
    }


    /**
     * Called periodically in the subsystem.
     */
    override fun periodic() {
        recordOutput("Drivetrain(Swerve)/CurrentModuleStates", SwerveModuleState.struct, *currentModuleStates.toArray())
        recordOutput("Drivetrain(Swerve)/DistanceTraveledMeters", distanceTraveled.inUnit(meters))
        recordOutput("Drivetrain(Swerve)/OverallVelocityMetersPerSec",velocity.inUnit(meters/seconds))
    }


}