package frc.robot.hardware.subsystems.odometry

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.hardware.subsystems.swervedrive.SwerveEncoders
import frc.chargers.hardware.subsystems.swervedrive.SwerveMotors
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.delay
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.external.frc6995.NomadAprilTagUtil
import frc.robot.hardware.subsystems.odometry.lowlevel.GyroOdometryIO
import frc.robot.hardware.subsystems.odometry.lowlevel.ModuleOdometryIO
import frc.robot.hardware.subsystems.odometry.lowlevel.OdometryTimestampsIO
import org.littletonrobotics.junction.Logger
import kotlin.math.PI


@Suppress("unused")
class ThreadedPoseMonitor(
    startingPose: UnitPose2d = UnitPose2d(),
    kinematics: SwerveDriveKinematics,
    private val hardwareData: SwerveHardwareData,
    private val navX: ChargerNavX,
    turnMotors: SwerveMotors<ChargerSparkMax>,
    driveMotors: SwerveMotors<ChargerTalonFX>,
    absoluteEncoders: SwerveEncoders<PositionEncoder>,
    private val visionPoseSuppliers: MutableList<VisionPoseSupplier> = mutableListOf()
): SubsystemBase(), RobotPoseMonitor {

    private val topLeftOdoSource = ModuleOdometryIO(
        "TopLeftModule",
        hardwareData,
        turnMotors.topLeft,
        driveMotors.topLeft,
        absoluteEncoders.topLeft
    )

    private val topRightOdoSource = ModuleOdometryIO(
        "topRightModule",
        hardwareData,
        turnMotors.topRight,
        driveMotors.topRight,
        absoluteEncoders.topRight
    )

    private val bottomLeftOdoSource = ModuleOdometryIO(
        "bottomLeftModule",
        hardwareData,
        turnMotors.bottomLeft,
        driveMotors.bottomLeft,
        absoluteEncoders.bottomLeft
    )

    private val bottomRightOdoSource = ModuleOdometryIO(
        "bottomRightModule",
        hardwareData,
        turnMotors.bottomRight,
        driveMotors.bottomRight,
        absoluteEncoders.bottomRight
    )

    private val gyroHeadingSource = GyroOdometryIO(navX)

    private val timestampsSource = OdometryTimestampsIO()

    private var previousHeading = 0.degrees


    init {
        OdometryThread.getInstance().start()
        // waits, then burns flashes for motors
        delay(0.02.seconds)
        topLeftOdoSource.burnMotorFlashes()
        topRightOdoSource.burnMotorFlashes()
        bottomLeftOdoSource.burnMotorFlashes()
        bottomRightOdoSource.burnMotorFlashes()
    }


    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(0.0),
        Array(4) { SwerveModulePosition() },
        startingPose.siValue
    )

    private var currentModulePositions = Array(4) { SwerveModulePosition() }

    private val indexOutOfBoundsAlert =
        Alert.warning(text = "It looks like the amount of timestamps recorded is greater than the amount of readings.")


    override val robotPose: UnitPose2d
        get() = poseEstimator.estimatedPosition.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d) {
        poseEstimator.resetPosition(
            navX.heading.asRotation2d(),
            currentModulePositions,
            pose.siValue
        )

        navX.zeroHeading(pose.rotation)
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {
        visionPoseSuppliers.addAll(visionSystems)
    }


    override fun periodic() {
        val timestamps = timestampsSource.timestamps

        require(timestamps.isNotEmpty()) {
            "There are no timestamps being recorded."
        }

        repeat(timestamps.size) { i ->
            try {
                currentModulePositions = a[
                    SwerveModulePosition(
                        topLeftOdoSource.wheelPositions[i].inUnit(meters),
                        topLeftOdoSource.wheelDirections[i].asRotation2d()
                    ),
                    SwerveModulePosition(
                        topRightOdoSource.wheelPositions[i].inUnit(meters),
                        topRightOdoSource.wheelDirections[i].asRotation2d()
                    ),
                    SwerveModulePosition(
                        bottomLeftOdoSource.wheelPositions[i].inUnit(meters),
                        bottomLeftOdoSource.wheelDirections[i].asRotation2d()
                    ),
                    SwerveModulePosition(
                        bottomRightOdoSource.wheelPositions[i].inUnit(meters),
                        bottomRightOdoSource.wheelDirections[i].asRotation2d()
                    )
                ]

                if (hardwareData.couplingRatio != null) {
                    currentModulePositions.forEach {
                        // applies coupling ratio offsets; not tested atm
                        // rotations is actually Rotation2d.getRotations() and not the kmeasure extension property
                        it.distanceMeters -=
                            (it.angle.rotations * hardwareData.couplingRatio) * PI * hardwareData.wheelDiameter.inUnit(meters)
                    }
                }

                poseEstimator.updateWithTime(
                    timestamps[i].inUnit(seconds),
                    gyroHeadingSource.gyroReadings[i].asRotation2d(),
                    currentModulePositions
                )
            } catch (_: IndexOutOfBoundsException) {
                indexOutOfBoundsAlert.active = true
            }
        }

        // if gyro is rotating too fast, do not add vision measurements
        if (gyroHeadingSource.gyroReadings.isNotEmpty()){
            val averageHeading = gyroHeadingSource.gyroReadings.average()
            val rotationRate = abs(averageHeading - previousHeading) / ChargerRobot.LOOP_PERIOD
            previousHeading = averageHeading
            if (rotationRate > 720.degrees / 1.seconds){
                return
            }
        }

        /*
        Sends all pose data to the pose estimator.

        This is currently not updated at a high frequency; this might change soon.
         */
        for (visionPoseSupplier in visionPoseSuppliers) {
            for (poseEstimate in visionPoseSupplier.robotPoseEstimates) {
                if (poseEstimate.value.distanceTo(robotPose) < 0.8.meters){
                    val stdDevVector = NomadAprilTagUtil.calculateVisionUncertainty(
                        poseEstimate.value.x.siValue,
                        navX.heading.asRotation2d(),
                        visionPoseSupplier.cameraYaw.asRotation2d(), // each pose supplier stores the camera yaw for std dev purposes
                    )

                    poseEstimator.addVisionMeasurement(
                        poseEstimate.value.inUnit(meters),
                        poseEstimate.timestamp.inUnit(seconds),
                        stdDevVector
                    )
                }
            }
        }

        Logger.recordOutput("ThreadedPoseEstimator/Pose2d", Pose2d.struct, robotPose.siValue)
    }
}
