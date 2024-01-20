package frc.robot.hardware.subsystems.odometry

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import org.littletonrobotics.junction.Logger


@Suppress("unused")
class ThreadedPoseMonitor(
    private val io: OdometryIO,
    private val kinematics: SwerveDriveKinematics,
    startingPose: UnitPose2d = UnitPose2d(),
    private val useGyro: Boolean = true,
    vararg visionPoseSuppliers: VisionPoseSupplier
): RobotPoseMonitor, SubsystemBase() {
    private val wheelRadius = io.hardwareData.wheelDiameter / 2.0

    private val visionPoseSuppliers = mutableListOf(*visionPoseSuppliers)

    private var previousTLPosition = 0.meters
    private var previousTRPosition = 0.meters
    private var previousBLPosition = 0.meters
    private var previousBRPosition = 0.meters

    private var previousGyroHeading = 0.degrees

    private var innacurateReadingTimestamps: MutableList<Double> = mutableListOf()

    private fun logInnacurateReadings(){
        Logger.recordOutput("ThreadedPoseEstimator/innacurateReadingsTimestamps", innacurateReadingTimestamps.toDoubleArray())
    }

    init{
        logInnacurateReadings()
    }


    override var robotPose: UnitPose2d = startingPose
        private set

    override fun resetPose(pose: UnitPose2d) {
        robotPose = pose
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {
        visionPoseSuppliers.addAll(visionSystems)
    }

    override fun periodic() {
        val numSamples = arrayOf(
            io.topLeftWheelDirections,
            io.topLeftWheelPositions,

            io.topRightWheelPositions,
            io.topRightWheelDirections,

            io.bottomLeftWheelPositions,
            io.bottomLeftWheelDirections,

            io.bottomRightWheelPositions,
            io.bottomRightWheelDirections,

            io.gyroHeadings
        ).minOfOrNull { it.size } ?: error("No samples are being recorded.")

        fun <D: Dimension<*,*,*,*>> isInvalid(input: Quantity<D>): Boolean{
            val siValue = input.siValue
            return siValue.isNaN() || siValue == Double.POSITIVE_INFINITY || siValue == Double.NEGATIVE_INFINITY
        }

        repeat(numSamples) {i ->
            val currentTLPosition = io.topLeftWheelPositions[i] * wheelRadius
            val currentTRPosition = io.topRightWheelPositions[i] * wheelRadius
            val currentBLPosition = io.bottomLeftWheelPositions[i]  * wheelRadius
            val currentBRPosition = io.bottomRightWheelPositions[i] * wheelRadius

            if (
                isInvalid(currentTLPosition) ||
                isInvalid(currentTRPosition) ||
                isInvalid(currentBLPosition) ||
                isInvalid(currentBRPosition)
            ){
                innacurateReadingTimestamps.add(Timer.getFPGATimestamp())
                logInnacurateReadings()
                return@repeat
            }


            val currentTLPositionDelta = currentTLPosition - previousTLPosition
            val currentTRPositionDelta = currentTRPosition - previousTRPosition
            val currentBLPositionDelta = currentBLPosition - previousBLPosition
            val currentBRPositionDelta = currentBRPosition - previousBRPosition

            if (
                abs(currentTLPositionDelta) > 0.5.meters ||
                abs(currentTRPositionDelta) > 0.5.meters ||
                abs(currentBLPositionDelta) > 0.5.meters ||
                abs(currentBRPositionDelta) > 0.5.meters
            ){
                innacurateReadingTimestamps.add(Timer.getFPGATimestamp())
                logInnacurateReadings()
                return@repeat
            }

            // angles are already zeroed off of absolute encoders; no delta calc nessecary
            val currentTLAngle = io.topLeftWheelDirections[i]
            val currentTRAngle = io.topRightWheelDirections[i]
            val currentBLAngle = io.bottomLeftWheelDirections[i]
            val currentBRAngle = io.bottomRightWheelDirections[i]

            previousTLPosition = currentTLPosition
            previousTRPosition = currentTRPosition
            previousBLPosition = currentBLPosition
            previousBRPosition = currentBRPosition

            val twist = kinematics.toTwist2d(
                SwerveModulePosition( currentTLPositionDelta.siValue, currentTLAngle.asRotation2d()),
                SwerveModulePosition( currentTRPositionDelta.siValue, currentTRAngle.asRotation2d()),
                SwerveModulePosition( currentBLPositionDelta.siValue, currentBLAngle.asRotation2d()),
                SwerveModulePosition( currentBRPositionDelta.siValue, currentBRAngle.asRotation2d()),
            )

            if (useGyro){
                val currentHeading = io.gyroHeadings[i]
                val deltaHeading = currentHeading - previousGyroHeading
                previousGyroHeading = currentHeading
                twist.dtheta = deltaHeading.inUnit(radians)
            }

            robotPose = UnitPose2d(robotPose.siValue.exp(twist))
        }
        Logger.recordOutput("ThreadedPoseEstimator/pose", Pose2d.struct, robotPose.siValue)
    }

}