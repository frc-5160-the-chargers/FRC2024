@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.vision.photonvision.ChargerPhotonCamera
import frc.robot.constants.ROBOT_TO_APRILTAG_PHOTON_CAM

class TestBoardRobotContainer: ChargerRobotContainer() {



    private val testPhotonCam = ChargerPhotonCamera("Photon Webcam", ROBOT_TO_APRILTAG_PHOTON_CAM)

    private val aprilTagPipeline =
        testPhotonCam
            .AprilTagPipeline(0, LoggableInputsProvider("AprilTagTesting"), usePoseEstimation = true)

    /*
    private val mlPipeline =
        testPhotonCam
            .ObjectPipeline(1, LoggableInputsProvider("MLTesting"))


     */

    val motor = TalonFX(7)

    override val autonomousCommand: Command
        get() = buildCommand {
            loopFor(3.seconds){
                println("running")
                motor.set(0.15)
            }
        }







}