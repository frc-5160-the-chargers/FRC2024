@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.commands

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.*
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.wpilibextensions.fpgaTimestamp


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
 * A utility function for creating [InstantCommand]s in a more kotlin-friendly way.
 */
public fun InstantCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand =
    InstantCommand(toRun, *subsystems)

/**
 * A utility function for creating [InstantCommand]s in a more kotlin-friendly way.
 */
public fun runOnceCommand(vararg subsystems: Subsystem, toRun: () -> Unit): InstantCommand =
    InstantCommand(toRun, *subsystems)

/**
 * A utility function for creating [DeferredCommand]s in a more kotlin-friendly way.
 */
public fun DeferredCommand(subsystems: Set<Subsystem>, getCommand: () -> Command): DeferredCommand =
    DeferredCommand(getCommand, subsystems)



/**
 * A utility function for creating [RunCommand]s in a more kotlin-friendly way.
 */
public fun RunCommand(vararg subsystems: Subsystem, toRun: () -> Unit): RunCommand =
    RunCommand(toRun, *subsystems)

/**
 * A utility function for creating [RunCommand]s in a more kotlin-friendly way.
 */
public fun loopCommand(vararg subsystems: Subsystem, toRun: () -> Unit): RunCommand =
    RunCommand(toRun, *subsystems)

/**
 * A utility function for setting the default [Command] of a [Subsystem] to a [RunCommand]
 * in a shorter and easier way.
 *
 * @see Subsystem.setDefaultCommand
 */
public fun <S: Subsystem> S.setDefaultRunCommand(
    vararg requirements: Subsystem,
    endBehavior: S.(Boolean) -> Unit = {},
    toRun: S.() -> Unit
){
    defaultCommand = RunCommand({toRun()}, this, *requirements).finallyDo{ interrupted -> endBehavior(interrupted) }
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


/**
 * Makes a command repeat for a certain amount of time
 */
public fun Command.repeatFor(time: Time): Command =
    ParallelRaceGroup(
        repeatedly(),
        WaitCommand(time.inUnit(seconds))
    )

/**
 * Makes the command repeat over and over while a condition is true.
 */
public fun Command.repeatWhile(condition: () -> Boolean): Command = ParallelRaceGroup(
    repeatedly(),
    object: Command(){
        override fun isFinished(): Boolean = !condition()
    }
)

/**
 * Makes a command repeat for a certain amount of times.
 */
public fun Command.repeat(numTimes: Int): Command = buildCommand {
    for (i in 1..numTimes){
        +this@repeat
    }
}

/**
 * Runs a function block before the command starts.
 */
public fun Command.beforeStarting(vararg subsystems: Subsystem, toRun: () -> Unit): Command =
    beforeStarting(toRun,*subsystems)



