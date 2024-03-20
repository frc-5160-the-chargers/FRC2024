package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d

class SwervePoseMonitor(
    private val drivetrain: EncoderHolonomicDrivetrain,
    visionEstimators: List<VisionPoseSupplier>,
    startingPose: UnitPose2d
): SubsystemBase(), RobotPoseMonitor {
    private val poseEstimator = SwerveDrivePoseEstimator(
        drivetrain.kinematics,
        Rotation2d(0.0),
        drivetrain.modulePositions.toTypedArray(),
        Pose2d()
    )

    private var gyroHeading: Angle = Angle(0.0)

    private val visionEstimators: MutableList<VisionPoseSupplier> = mutableListOf()

    private val robotObject = ChargerRobot.FIELD.getObject(drivetrain.logName)

    private var previousWheelTravelDistances: MutableList<Distance> =
        drivetrain.modulePositions.map{ it.distanceMeters.ofUnit(meters) }.toMutableList()


    init{
        addPoseSuppliers(*visionEstimators.toTypedArray())
        resetPose(startingPose)
    }


    override val robotPose: UnitPose2d
        get() = poseEstimator.estimatedPosition.ofUnit(meters)

    override fun resetPose(pose: UnitPose2d) {
        if (drivetrain.gyro is ZeroableHeadingProvider){
            drivetrain.gyro.zeroHeading(pose.rotation)
            gyroHeading = drivetrain.gyro.heading
        }else{
            gyroHeading = pose.rotation
        }

        poseEstimator.resetPosition(
            gyroHeading.asRotation2d(),
            drivetrain.modulePositions.toTypedArray(),
            pose.inUnit(meters)
        )
    }

    override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {
        visionEstimators.addAll(visionSystems)
    }


    override fun periodic(){
        if (drivetrain.gyro != null){
            gyroHeading = drivetrain.gyro.heading
        }else{
            val wheelDeltas = drivetrain.modulePositions.mapIndexed{i, value ->
                val delta = value.distanceMeters - previousWheelTravelDistances[i].inUnit(meters)
                previousWheelTravelDistances[i] = value.distanceMeters.ofUnit(meters)
                SwerveModulePosition(delta, value.angle)
            }

            gyroHeading += drivetrain
                .kinematics
                .toTwist2d(*wheelDeltas.toTypedArray())
                .dtheta
                .ofUnit(radians)
        }

        poseEstimator.update(
            gyroHeading.asRotation2d(),
            drivetrain.modulePositions.toTypedArray()
        )

        for (visionEstimator in visionEstimators){
            for (visionEstimate in visionEstimator.robotPoseEstimates){
                poseEstimator.addVisionMeasurement(
                    visionEstimate.value.inUnit(meters),
                    visionEstimate.timestamp.inUnit(seconds)
                )
            }
        }

        robotObject.pose = poseEstimator.estimatedPosition
    }
}