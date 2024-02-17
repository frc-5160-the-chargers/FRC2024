@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.units.seconds
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.vision.limelight.ChargerLimelight
import frc.chargers.hardware.sensors.vision.photonvision.ChargerPhotonCamera
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d

class TestBoardRobotContainer: ChargerRobotContainer() {
    private val testPhotonCam = ChargerPhotonCamera("Arducam_OV9281", UnitTransform3d())

    private val photonTagPipeline =
        testPhotonCam
            .AprilTagPipeline(0, LoggableInputsProvider("PhotonAprilTagPipeline"), usePoseEstimation = true)

    private val testLimelight = ChargerLimelight(robotToCamera = UnitTransform3d())

    private val limelightTagPipeline =
        testLimelight
            .AprilTagPipeline(0, LoggableInputsProvider("LimelightAprilTagPipeline"), usePoseEstimation = false)




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