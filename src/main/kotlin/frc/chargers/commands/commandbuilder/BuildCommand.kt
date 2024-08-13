package frc.chargers.commands.commandbuilder

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.chargers.commands.withExtraRequirements
import frc.chargers.commands.withLogging
import kotlin.internal.LowPriorityInOverloadResolution

/**
 * The entry point for the CommandBuilder DSL (Domain Specific Language).
 *
 * See [here](https://kotlinlang.org/docs/type-safe-builders.html#how-it-works)
 * for an explanation of DSLs and how they are built.
 *
 * Example usage:
 * ```
 * val armCommand: Command = buildCommand{
 *      require(arm) // adds requirements across the entire command group
 *
 *      // equivalent to an InstantCommand within a SequentialCommandGroup
 *      runOnce{ // requirements of the entire command group can also be added here
 *          armSubsystem.resetEncoders()
 *      }
 *
 *      // variable initialization is allowed, since the block is essentially a lambda function
 *      val pidController = UnitSuperPIDController(...)
 *      // getOnceDuringRun acts as a value that is refreshed once every time the command is scheduled;
 *      // order is synchronous
 *      val armCurrentPosition by getOnceDuringRun{arm.distalAngle}
 *
 *      // loops through the block below until the condition is met
 *      loopUntil(condition = { abs(pidController.target - arm.distalAngle) < 0.1 }){
 *          armSubsystem.setDistalVoltage(pidController.calculateOutput())
 *      }.modify{ it.unless(booleanSupplier).withTimeout(5) } // All command factory methods must be called from a modify() function call
 *
 * }
 * ```
 * @param name The name of the buildCommand(defaults to "Generic BuildCommand").
 * @param log If true, will log the individual commands that are part of the DSL. Defaults to false.
 * @param block The entry point to the DSL. Has the context of [CommandBuilder].
 */
inline fun buildCommand(
    name: String = "Unnamed BuildCommand",
    log: Boolean = false,
    block: BuildCommandScope.() -> Unit
): Command {
    val builder = BuildCommandScope().apply(block)

    val subCommands = builder.commands.map{
        if (log) it.withLogging("$name/${it.name}") else it
    }.toTypedArray()

    var command: Command = SequentialCommandGroup(*subCommands)
        .withName(name)
        .until(builder.context::commandStopped)
        .finallyDo(builder.endBehavior)
        .withLogging("$name/overallCommand")

    if (builder.requirements.size > 0){
        command = command.withExtraRequirements(*builder.requirements.toTypedArray())
    }

    builder.addingCommandsLocked = true
    builder.addingRequirementsLocked = true

    return command
}

/**
* Creates a [buildCommand] that automatically requires a subsystem.
*/
@LowPriorityInOverloadResolution
inline fun Subsystem.buildCommand(
    name: String = "Generic BuildCommand of " + getName(),
    logIndividualCommands: Boolean = false,
    block: BuildCommandScope.() -> Unit
): Command =
    buildCommand(name, logIndividualCommands, block)
        .withExtraRequirements(this@buildCommand)


/**
 * A scope exclusive to [buildCommand]; this contains things like end behavior and command requirements
 * which aren't used in other places where a [CommandBuilder] scope is asked for
 * (like runSequentially, runParallelUntilAllFinish, etc.)
 */
@CommandBuilderMarker
class BuildCommandScope: CommandBuilder() {
    val requirements: LinkedHashSet<Subsystem> = linkedSetOf()

    var addingRequirementsLocked: Boolean = false

    var endBehavior: (Boolean) -> Unit = {}

    /**
     * Adds subsystems that are required across the entire [buildCommand].
     */
    fun require(vararg requirements: Subsystem){
        if (addingRequirementsLocked){
            error("""
                It looks like you are attempting to add requirements to the command while it is running.
                This is not allowed; you must call addRequirements() within the block of the buildCommand, and not in a CodeBlockContext.
                
                For instance:
                buildCommand{
                    // correct way to do it; outside of any block
                    require(...)
                    
                    runOnce{
                        // does not compile
                        require(...)
                        // compiles, but will NOT WORK!
                        this@buildCommand.addRequirements(...)
                    }
                }
            """.trimIndent())
        }else{
            this.requirements.addAll(requirements)
        }
    }

    /**
     * Alternative syntax for adding requirements,
     * that more closely follows the syntax of base commands.
     */
    @Suppress("unused")
    fun addRequirements(vararg requirements: Subsystem){
        require(*requirements)
    }

    /**
     * Runs the function block when the [buildCommand] is finished.
     */
    fun onEnd(run: (Boolean) -> Unit){
        endBehavior = run
    }
}