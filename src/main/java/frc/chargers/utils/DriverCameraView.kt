@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance

/**
 * Wraps WPILib's [CameraServer]; being used to handle driver cameras.
 */
public class DriverCameraView(
    totalDriverCams: Int = 1,
    private val defaultResWidth: Int,
    private val defaultResHeight: Int
){
    private val driverCameras = List(totalDriverCams){ i ->
        CameraServer.startAutomaticCapture(i).apply{
            setResolution(defaultResWidth,defaultResHeight)
        }
    }

    private val cameraSelector: NetworkTableEntry =
        NetworkTableInstance
            .getDefault()
            .getTable("")
            .getEntry("CameraSelection")

    public fun switchToCamera(index: Int){
        cameraSelector.setString(driverCameras[index].name)
    }

    public fun setResolutionOf(cameraIndex: Int, width: Int, height: Int){
        driverCameras[cameraIndex].setResolution(width, height)
    }

    public fun setResolution(width: Int, height: Int){
        driverCameras.forEach{it.setResolution(width, height)}
    }
}