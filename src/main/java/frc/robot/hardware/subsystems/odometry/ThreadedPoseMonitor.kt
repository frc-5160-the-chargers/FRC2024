package frc.robot.hardware.subsystems.odometry

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.sensors.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.subsystems.swervedrive.SwerveEncoders
import frc.chargers.hardware.subsystems.swervedrive.SwerveMotors
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.external.frc6995.NomadApriltagUtil
import frc.robot.hardware.subsystems.odometry.lowlevel.GyroOdometryIO
import frc.robot.hardware.subsystems.odometry.lowlevel.ModuleOdometryIO
import frc.robot.hardware.subsystems.odometry.lowlevel.OdometryTimestampsIO
import org.littletonrobotics.junction.Logger


@Suppress("unused")
class ThreadedPoseMonitor(
    startingPose: UnitPose2d = UnitPose2d(),

    kinematics: SwerveDriveKinematics,
    hardwareData: SwerveHardwareData,

    private val navX: ChargerNavX,

    turnMotors: SwerveMotors<ChargerSparkMax>,
    driveMotors: SwerveMotors<ChargerSparkMax>,
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

    private val gyroOdoSource = GyroOdometryIO(navX)

    private val timestampsSource = OdometryTimestampsIO()



    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(0.0),
        Array(4){SwerveModulePosition()},
        startingPose.siValue
    )

    private var currentModulePositions = Array(4){SwerveModulePosition()}





    override val robotPose: UnitPose2d
        get() = poseEstimator.estimatedPosition.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d) {
        poseEstimator.resetPosition(
            navX.heading.asRotation2d(),
            currentModulePositions,
            pose.siValue
        )
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier){
        visionPoseSuppliers.addAll(visionSystems)
    }





    override fun periodic(){
        val timestamps = timestampsSource.timestamps

        repeat(timestamps.size){ i ->
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

            poseEstimator.updateWithTime(
                timestamps[i].inUnit(seconds),
                gyroOdoSource.gyroReadings[i].asRotation2d(),
                currentModulePositions
            )
        }


        /*
        Sends all pose data to the pose estimator.

        This is currently not updated at a high frequency; this might change soon.
         */
        visionPoseSuppliers.forEach {
            val measurement = it.robotPoseEstimate
            if (measurement != null) {
                val stdDevVector = NomadApriltagUtil.calculateVisionUncertainty(
                    measurement.value.x.siValue,
                    navX.heading.asRotation2d(),
                    it.cameraYaw.asRotation2d(),
                )
                poseEstimator.addVisionMeasurement(
                    measurement.value.inUnit(meters),
                    measurement.timestamp.inUnit(seconds),
                    stdDevVector
                )
            }
        }

        Logger.recordOutput("ThreadedPoseEstimator/pose", Pose2d.struct, robotPose.siValue)
    }
}
