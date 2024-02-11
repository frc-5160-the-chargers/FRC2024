@file:Suppress("unused")
package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.InstantCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.vision.photonvision.ChargerPhotonCamera
import frc.robot.constants.ROBOT_TO_APRILTAG_PHOTON_CAM

class TestBoardRobotContainer: ChargerRobotContainer() {


    private val testPhotonCam = ChargerPhotonCamera("Photon Webcam", ROBOT_TO_APRILTAG_PHOTON_CAM)

    private val aprilTagPipeline =
        testPhotonCam
            .AprilTagPipeline(0, LoggableInputsProvider("AprilTagTesting"), usePoseEstimation = true)

    private val mlPipeline =
        testPhotonCam
            .ObjectPipeline(1, LoggableInputsProvider("MLTesting"))

    override val autonomousCommand: Command
        get() = InstantCommand { }

}