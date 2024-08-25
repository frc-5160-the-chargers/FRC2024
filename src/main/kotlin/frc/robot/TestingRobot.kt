package frc.robot

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop
import frc.chargers.commands.commandbuilder.Request
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.framework.ChargerRobot

class TestingRobot: ChargerRobot() {
    override fun robotInit(){
        teleop().whileTrue(
            buildCommand {
                loopFor(5.seconds){
                    println("hello!")
                    return@loopFor Request.BREAK
                }

                runOnce{
                    println("bye!")
                }
            }
        )
    }
}