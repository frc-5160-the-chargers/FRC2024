package frc.robot.hardware.subsystems.odometry

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.external.frc6328.MechanicalAdvantagePoseEstimator
import frc.external.frc6995.NomadApriltagUtil
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

    private val poseEstimator = MechanicalAdvantagePoseEstimator(
        VecBuilder.fill(0.003, 0.003, 0.00001)
    )
    private val visionPoseSuppliers = mutableListOf(*visionPoseSuppliers)
    private val visionUpdates = mutableListOf<MechanicalAdvantagePoseEstimator.TimestampedVisionUpdate>()

    private var noVisionPose = UnitPose2d()


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
        poseEstimator.resetPose(startingPose.siValue)
    }

    override val robotPose: UnitPose2d = poseEstimator.latestPose.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d) {
        poseEstimator.resetPose(pose.siValue)
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

        var numActualSamples = 0
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
                abs(currentTLPositionDelta) > 1.meters ||
                abs(currentTRPositionDelta) > 1.meters ||
                abs(currentBLPositionDelta) > 1.meters ||
                abs(currentBRPositionDelta) > 1.meters
            ){
                innacurateReadingTimestamps.add(Timer.getFPGATimestamp())
                logInnacurateReadings()
                return@repeat
            }
            numActualSamples++

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


            poseEstimator.addDriveData(
                Timer.getFPGATimestamp(),
                twist
            )

            noVisionPose = UnitPose2d(noVisionPose.siValue.exp(twist))
        }

        /*
        Sends all pose data to the pose estimator.
         */
        if (visionPoseSuppliers.size > 0) {
            visionUpdates.clear()
            visionPoseSuppliers.forEach {
                val measurement = it.robotPoseEstimate
                if (measurement != null) {
                    val stdDevVector = NomadApriltagUtil.calculateVisionUncertainty(
                        measurement.value.x.siValue,
                        heading.asRotation2d(),
                        it.cameraYaw.asRotation2d(),
                    )
                    visionUpdates.add(
                        MechanicalAdvantagePoseEstimator.TimestampedVisionUpdate(
                            measurement.timestamp.inUnit(seconds),
                            measurement.value.inUnit(meters),
                            stdDevVector
                        )
                    )
                }
            }
            if (visionUpdates.size != 0) {
                poseEstimator.addVisionData(visionUpdates)
            }
        }


        Logger.recordOutput("ThreadedPoseEstimator/debugNovisionPose", Pose2d.struct, noVisionPose.siValue)
        Logger.recordOutput("ThreadedPoseEstimator/pose", Pose2d.struct, robotPose.siValue)
        Logger.recordOutput("ThreadedPoseEstimator/numActualSamples", numActualSamples)
        Logger.recordOutput("ThreadedPoseEstimator/numIntendedSamples", numSamples)
    }

}
