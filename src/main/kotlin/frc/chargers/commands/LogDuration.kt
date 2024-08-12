@file:Suppress("unused")
package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.chargers.wpilibextensions.fpgaTimestamp

/**
 * Logs the duration of a command.
 */
fun Command.logDuration(logName: String = this.name): Command =
    object: WrapperCommand(this){
        private var startTime = fpgaTimestamp()

        override fun initialize(){
            startTime = fpgaTimestamp()
            this@logDuration.initialize()
        }

        override fun end(interrupted: Boolean){
            this@logDuration.end(interrupted)
            println("Command with name $logName has finished with time " + (fpgaTimestamp() - startTime).siValue + " seconds.")
        }
    }