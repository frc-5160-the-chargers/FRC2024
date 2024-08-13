@file:Suppress("unused")
package frc.chargers.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.chargers.framework.Loggable
import frc.chargers.wpilibextensions.fpgaTimestamp

/**
 * Logs the duration of a command, as well as whether it is running or not
 */
fun Command.withLogging(logName: String = this.name): Command =
    object: WrapperCommand(this), Loggable {
        private var startTime = fpgaTimestamp()
        override val namespace = "Commands/$logName"

        init{
            log("IsRunning", false)
            log("ExecutionTime", 0.seconds)
        }

        override fun initialize(){
            startTime = fpgaTimestamp()
            this@withLogging.initialize()
            log("IsRunning", true)
        }

        override fun end(interrupted: Boolean){
            this@withLogging.end(interrupted)
            log("ExecutionTime", fpgaTimestamp() - startTime)
            log("IsRunning", false)
        }
    }