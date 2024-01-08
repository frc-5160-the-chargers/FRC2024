package frc.robot.hardware.subsystems.odometry

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.sensors.RobotPoseMonitor
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d

@Suppress("unused")
class ThreadedPoseMonitor(
    private val io: OdometryIO,
    private val drivetrain: EncoderHolonomicDrivetrain,
    startingPose: UnitPose2d = UnitPose2d(),
    private val useGyro: Boolean = true,
    vararg visionPoseSuppliers: VisionPoseSupplier
): RobotPoseMonitor, SubsystemBase() {
    private val wheelRadius = drivetrain.hardwareData.wheelDiameter / 2.0

    private val visionPoseSuppliers = mutableListOf(*visionPoseSuppliers)

    private var previousTLPosition = 0.meters
    private var previousTRPosition = 0.meters
    private var previousBLPosition = 0.meters
    private var previousBRPosition = 0.meters



    init{
        if (RobotBase.isSimulation()){
            error("Threaded pose estimation doesn't work in sim; sorry!")
        }
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

        repeat(numSamples) {i ->
            val currentTLPosition = io.topLeftWheelPositions[i] * wheelRadius
            val currentTRPosition = io.topRightWheelPositions[i] * wheelRadius
            val currentBLPosition = io.bottomLeftWheelPositions[i]  * wheelRadius
            val currentBRPosition = io.bottomRightWheelPositions[i] * wheelRadius

            val currentTLPositionDelta = currentTLPosition - previousTLPosition
            val currentTRPositionDelta = currentTRPosition - previousTRPosition
            val currentBLPositionDelta = currentBLPosition - previousBLPosition
            val currentBRPositionDelta = currentBRPosition - previousBRPosition

            // angles are already zeroed off of absolute encoders; no delta calc nessecary
            val currentTLAngle = io.topLeftWheelDirections[i]
            val currentTRAngle = io.topRightWheelDirections[i]
            val currentBLAngle = io.bottomLeftWheelDirections[i]
            val currentBRAngle = io.bottomRightWheelDirections[i]

            previousTLPosition = currentTLPosition
            previousTRPosition = currentTRPosition
            previousBLPosition = currentBLPosition
            previousBRPosition = currentBRPosition

            val twist = drivetrain.kinematics.toTwist2d(
                SwerveModulePosition( currentTLPositionDelta.siValue, currentTLAngle.asRotation2d()),
                SwerveModulePosition( currentTRPositionDelta.siValue, currentTRAngle.asRotation2d()),
                SwerveModulePosition( currentBLPositionDelta.siValue, currentBLAngle.asRotation2d()),
                SwerveModulePosition( currentBRPositionDelta.siValue, currentBRAngle.asRotation2d()),
            )

            if (useGyro){
                twist.dtheta = io.gyroHeadings[i].inUnit(radians)
            }

            robotPose = UnitPose2d(robotPose.siValue.exp(twist))
        }
    }

}