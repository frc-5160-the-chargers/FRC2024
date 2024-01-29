@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.robotposition

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.Filesystem
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import java.io.File

/**
 * Represents a pose monitor which combines wheel odometry(and optionally vision data)
 * to produce a robot pose.
 *
 * This class should not be implemented by vision cameras or sensors, as their pose results
 * are not always accurate, and thus the nullable parameters are necessary.
 */
public interface RobotPoseMonitor: HeadingProvider {
    public val robotPose: UnitPose2d

    public fun resetPose(pose: UnitPose2d)

    public fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier)


    public fun zeroPose(): Unit = resetPose(UnitPose2d())

    override val heading: Angle get() = robotPose.rotation


    public fun resetToPathplannerTrajectory(pathName: String){
        val path = PathPlannerPath.fromPathFile(pathName)
        resetPose(path.startingDifferentialPose.ofUnit(meters))
    }

    // when you read the starting pose of a pathplanner path,
    // the heading value is not equivalent.
    // thus, we need to manually parse the .chor file in order to obtain the proper heading(lol).
    public fun resetToChoreoTrajectory(pathName: String){
        val translation = PathPlannerPath.fromChoreoTrajectory(pathName).startingDifferentialPose.translation

        // to not require the choreolib dep, we use some file readline shenanigans
        val file = File(Filesystem.getDeployDirectory(), "choreo/$pathName.traj")
        require(file.exists()){ "The pathname specified does not exist!" }
        val fileReader = file.bufferedReader()
        try{
            // skips to the first path heading reference
            repeat(5){
                fileReader.readLine()
            }

            val headingData = fileReader.readLine()

            val startIndex = headingData.indexOf("\"heading\": ")
            val endIndex = headingData.indexOf(",")

            if (startIndex == -1 || endIndex == -1){
                error("") // causes code to jump to exception block
            }

            resetPose(
                // pose2d wrapper w/ units support
                UnitPose2d(
                    translation.ofUnit(meters),
                    headingData.substring(startIndex, endIndex).toDouble().ofUnit(radians)
                )
            )
        }catch(e: Throwable){
            error("It looks like there is no heading data to be found in your path file.")
        }
    }

}

