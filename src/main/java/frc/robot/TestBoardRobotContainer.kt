@file:Suppress("unused")
package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.subsystems.robotposition.RobotPoseMonitor
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.hardware.subsystems.vision.VisionManager

class TestBoardRobotContainer: ChargerRobotContainer() {

    private val vision = VisionManager(object: RobotPoseMonitor{
        override val robotPose: UnitPose2d get() = UnitPose2d()

        override fun resetPose(pose: UnitPose2d) {}

        override fun addPoseSuppliers(vararg visionSystems: VisionPoseSupplier) {}
    })


    override val autonomousCommand: Command
        get() = buildCommand {

        }
}