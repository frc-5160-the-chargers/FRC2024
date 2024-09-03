package frc.robot

import edu.wpi.first.wpilibj.RobotBase

/**
 * Do not edit this class at all.
 */
object Main {
    /**
     * Main initialization function. Do not perform any initialization here.
     */
    @JvmStatic
    fun main(args: Array<String>) {
        // @java-ignore
        RobotBase.startRobot(::getRobot)
    }
}
