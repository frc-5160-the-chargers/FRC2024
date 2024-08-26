@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem


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





