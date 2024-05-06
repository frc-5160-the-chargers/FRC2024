@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.chargers.wpilibextensions.fpgaTimestamp


/**
 * Logs the duration of a command.
 */
public fun Command.logDuration(logName: String = this.name): Command =
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



/**
 * Constructs a command with extra requirements.
 */
public fun Command.withExtraRequirements(vararg requirements: Subsystem): Command =
    object: Command(){
        override fun getName(): String = this@withExtraRequirements.name

        init{
            addRequirements(*requirements, *this.requirements.toTypedArray())
        }

        override fun initialize(){
            this@withExtraRequirements.initialize()
        }

        override fun execute(){
            this@withExtraRequirements.execute()
        }

        override fun end(interrupted: Boolean) {
            this@withExtraRequirements.end(interrupted)
        }

        override fun isFinished(): Boolean = this@withExtraRequirements.isFinished
    }


/**
 * A way to link up 2 commands in a more concise way.
 */
public infix fun Command.then(other: Command): Command =
    andThen(other)




