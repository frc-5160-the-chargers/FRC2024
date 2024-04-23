@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.utils.Measurement
import frc.chargers.utils.flipWhenNeeded
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asRotation2d
import frc.external.frc6995.NomadAprilTagUtil
import org.photonvision.EstimatedRobotPose
import java.io.File
import java.util.*


/**
 * Represents a pose monitor which combines wheel odometry(and optionally vision data)
 * to produce a robot pose.
 *
 * This class should not be implemented by vision cameras or sensors, as their pose results
 * are not always accurate, and thus the nullable parameters are necessary.
 */
interface RobotPoseMonitor: HeadingProvider, Subsystem {
    val robotPose: UnitPose2d

    fun resetPose(pose: UnitPose2d)

    fun addVisionMeasurement(measurement: Measurement<UnitPose2d>, stdDevs: Matrix<N3, N1>? = null)




    fun addVisionMeasurement(measurement: Measurement<UnitPose2d>, cameraYaw: Angle){
        addVisionMeasurement(
            measurement,
            NomadAprilTagUtil.calculateVisionUncertainty(
                measurement.value.x.inUnit(meters),
                measurement.value.rotation.asRotation2d(),
                cameraYaw.asRotation2d()
            )
        )
    }

    fun addVisionMeasurement(photonEstimatedPose: Optional<EstimatedRobotPose>, cameraYaw: Angle){
        if (photonEstimatedPose.isPresent){
            addVisionMeasurement(
                photonEstimatedPose,
                NomadAprilTagUtil.calculateVisionUncertainty(
                    photonEstimatedPose.get().estimatedPose.x,
                    photonEstimatedPose.get().estimatedPose.rotation.toRotation2d(),
                    cameraYaw.asRotation2d()
                )
            )
        }
    }

    fun addVisionMeasurement(photonEstimatedPose: Optional<EstimatedRobotPose>, stdDevs: Matrix<N3, N1>? = null){
        if (photonEstimatedPose.isPresent){
            addVisionMeasurement(
                Measurement(
                    photonEstimatedPose.get().estimatedPose.toPose2d().ofUnit(meters),
                    photonEstimatedPose.get().timestampSeconds.ofUnit(seconds)
                )
            )
        }
    }

    fun zeroPose(){
        resetPose(UnitPose2d())
    }

    override val heading: Angle get() = robotPose.rotation

    /**
     * Resets the pose monitor's pose to a pathplanner path.
     */
    fun resetToPathPlannerPath(pathName: String, useHolonomicPose: Boolean){
        val path = PathPlannerPath.fromPathFile(pathName)
        if (useHolonomicPose){
            resetPose(path.previewStartingHolonomicPose.ofUnit(meters).flipWhenNeeded())
        }else{
            resetPose(path.startingDifferentialPose.ofUnit(meters).flipWhenNeeded())
        }
    }

    // when you read the starting pose of a pathplanner path,
    // the heading value is not equivalent.
    // thus, we need to manually parse the .chor file in order to obtain the proper heading(lol).
    /**
     * Resets the pose monitor's pose to a choreo trajectory.
     */
    fun resetToChoreoPath(pathName: String){
        val translation = PathPlannerPath.fromChoreoTrajectory(pathName).startingDifferentialPose.translation

        // to not require the choreolib dep, we use some file readline shenanigans
        val file = File(Filesystem.getDeployDirectory(), "choreo/$pathName.traj")
        require(file.exists()){ "The pathname specified does not exist!" }
        val fileReader = file.bufferedReader()
        // skips to the first path heading reference
        repeat(5){
            fileReader.readLine()
        }

        val headingData = fileReader.readLine()

        val startIndex = headingData.indexOf(":")
        val endIndex = headingData.indexOf(",")

        if (startIndex == -1 || endIndex == -1){
            error("") // causes code to jump to exception block
        }


        resetPose(
            // pose2d wrapper w/ units support
            UnitPose2d(
                translation.ofUnit(meters),
                headingData.substring(startIndex+2, endIndex).toDouble().ofUnit(radians)
            ).flipWhenNeeded()
        )
    }
}

