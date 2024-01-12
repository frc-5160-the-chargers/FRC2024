@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.commands.commandbuilder

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj2.command.*
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.RunCommand
import frc.chargers.commands.then
import frc.chargers.commands.withExtraRequirements
import org.littletonrobotics.junction.Logger
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty


/**
 * The entry point for the CommandBuilder DSL (Domain Specific Language).
 *
 * See [here](https://kotlinlang.org/docs/type-safe-builders.html#how-it-works)
 * for an explanation of DSLs and how they are built.
 *
 * Example usage:
 * ```
 * val armCommand: Command = buildCommand{
 *      // equivalent to an InstantCommand within a SequentialCommandGroup
 *      runOnce{
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
 *      }
 *
 * }
 * ```
 * @param name The name of the buildCommand(defaults to "Generic BuildCommand").
 * @param logIndividualCommands If true, will log the individual commands that are part of the DSL. Defaults to false.
 * @param shouldConvertCommandsToProxy If true, all the commands added to the [CommandBuilder] will become proxy commands:
 * which mean that the requirements of individual commands(like runOnce, loopForever, etc.)
 * aren't required through the entire buildCommand. Defaults to true; it is recommended to use
 * addRequirements for buildCommand-wide requirements instead of turning this option to false.
 * @param block The entry point to the DSL. Has the context of [CommandBuilder].
 */
public inline fun buildCommand(
    name: String = "Generic BuildCommand",
    logIndividualCommands: Boolean = false,
    shouldConvertCommandsToProxy: Boolean = true,
    block: CommandBuilder.() -> Unit
): Command{
    val builder = CommandBuilder().apply(block)

    val commandArray: Array<Command> = if (shouldConvertCommandsToProxy){
        builder.commands.map{
            val commandName = it.name
            return@map it.asProxy().withName(commandName)
        }.toTypedArray()
    }else{
        builder.commands.toTypedArray()
    }

    var command: Command = if(logIndividualCommands){
        loggedSequentialCommandGroup(name, *commandArray)
    }else{
        SequentialCommandGroup(*commandArray).withName(name)
    }.finallyDo(builder.endBehavior)

    if (builder.requirements.size > 0){
        command = command.withExtraRequirements(*builder.requirements.toTypedArray())
    }
    builder.addingCommandsLocked = true

    return command
}


/**
 * This "marker" serves to restrict the scope of the buildCommand DSL.
 *
 * See [here](https://kotlinlang.org/docs/type-safe-builders.html#scope-control-dslmarker)
 * for the purpose of this annotation class.
 */
@DslMarker
public annotation class CommandBuilderMarker

/**
 * The scope class responsible for governing the BuildCommand DSL.
 */
@CommandBuilderMarker
public class CommandBuilder{
    public var commands: LinkedHashSet<Command> = linkedSetOf() // LinkedHashSet keeps commands in order, but also ensures they're not added multiple times

    public val requirements: MutableList<Subsystem> = mutableListOf()

    public var endBehavior: (Boolean) -> Unit = {}

    public var addingCommandsLocked: Boolean = false

    public fun addCommand(c: Command){
        if (addingCommandsLocked){
            error(
                """
                It seems that you have attempted to add a command to the CommandBuilder during runtime. This is not allowed.
                Make sure that you don't have any nested runOnce, loopForever, loopUntil, etc. blocks 
                within another command-adding block, and make sure that all functions that are suffixed with 'Action' 
                are placed in the command builder block instead of a code block. For instance: 
                
                buildCommand{
                
                    runOnce{
                        // NOT ALLOWED
                        this@buildCommand.runOnce{
                            doSomething()
                        }
                        // Will not compile
                        runOnce{
                            doSomethingElse()
                        }
                        // NOT ALLOWED
                        drivetrain.driveStraightAction(...)
                    }
                    
                    // Intended behavior
                    drivetrain.driveStraightAction(...)
                } 
                """.trimIndent()
            )
        }else{
            commands.add(c)
        }
    }

    private fun removeCommand(c: Command){
        commands.remove(c)
    }

    
    /**
     * Adds a single command to be run until its completion.
     * Usage Example:
     * ```
     * buildCommand{
     *      +ArmCommand(2.0.radians, armSubsystem)
     * }
     * ```
     */
    public operator fun <C : Command> C.unaryPlus(): C{
        addCommand(this)
        return this
    }

    /**
     * Returns a command, then removes it from the set of commands within the [CommandBuilder].
     *
     * ```
     * buildCommand{
     *    val command by getOnceDuringRun{
     *      -runOnce{ doSomethingHere()}
     *    }
     * }
     * ```
     */
    public operator fun <C: Command> C.unaryMinus(): C{
        removeCommand(this)
        return this
    }


    /**
     * Adds subsystems that are required across the entire [buildCommand].
     */
    public fun addRequirements(vararg requirements: Subsystem){
        this.requirements.addAll(requirements)
    }

    /**
     * Runs the function block when the [buildCommand] is finished.
     */
    public inline fun onEnd(crossinline run: CodeBlockContext.(Boolean) -> Unit){
        endBehavior = { CodeBlockContext.run(it) }
    }



    /**
     * Adds a command that will run once and then complete.
     *
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run
     */
    public fun runOnce(vararg requirements: Subsystem, execute: CodeBlockContext.() -> Unit): InstantCommand =
        InstantCommand(*requirements) { CodeBlockContext.execute() }.also(::addCommand)


    /**
     * Adds a command that will run the command onTrue or only if a condition is met.
     */
    public fun runIf(condition: () -> Boolean, onTrue: Command, onFalse: Command): ConditionalCommand =
        ConditionalCommand(onTrue,onFalse,condition).also(::addCommand)

    /**
     * Adds a command that will run the appropriate mapped command, depending on the key given.
     *
     *
     * @param key: A lambda that gets a generic value, used for choosing an appropriate command.
     * @param commands: A map between the key and commands to be called.
     */
    public fun <T: Any> runWhen(key: () -> T, commands: Map<T, Command>): Command =
        SelectCommand(commands, key).also(::addCommand)


    /**
     * Runs a specific command out of many options when the correct index is passed in.
     */
    public fun runDependingOnIndex(
        indexSupplier: () -> Int,
        vararg commands: Command,
        block: CommandBuilder.() -> Unit = {}
    ): Command{
        var index = 0
        val allCommands = commands.associateBy { index.also{index++} }.toMutableMap()
        CommandBuilder().apply(block).commands.forEach{
            allCommands[index] = it
            index++
        }
        this.commands.removeAll(allCommands.values.toSet())

        return SelectCommand(allCommands, indexSupplier).also(::addCommand)
    }


    /**
     * Adds a command that will run *until* [condition] is met.
     *
     * @param condition the condition to be met
     * @param command the command to run until [condition] is met
     */
    public fun runUntil(condition: () -> Boolean, command: Command): ParallelRaceGroup =
        command.until { condition() }
            .also(::addCommand)

    /**
     * Adds a command that will run *until* [condition] is met.
     *
     * @param condition the condition to be met
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run until [condition] is met
     */
    public inline fun loopUntil(noinline condition: () -> Boolean, vararg requirements: Subsystem, crossinline execute: CodeBlockContext.() -> Unit): ParallelRaceGroup =
        runUntil(condition, RunCommand(*requirements) { CodeBlockContext.execute() })

    /**
     * Adds a command that will run *while* [condition] is true.
     *
     * @param condition the condition to be met
     * @param requirements the Subsystems this command uses
     * @param execute the code to be run
     */
    public inline fun loopWhile(crossinline condition: () -> Boolean, vararg requirements: Subsystem, noinline execute: CodeBlockContext.() -> Unit): ParallelRaceGroup =
        loopUntil({ !condition() }, *requirements, execute=execute)

    /**
     * Adds several commands that will run at the same time, all stopping as soon as one finishes.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelRaceGroup
     */
    public fun runParallelUntilOneFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        var command: Command = ParallelRaceGroup(*commandsSet.map{it.asProxy()}.toTypedArray()).finallyDo(builder.endBehavior)
        if (builder.requirements.size > 0){
            command = command.withExtraRequirements(*builder.requirements.toTypedArray())
        }
        builder.addingCommandsLocked = true
        return command.also(::addCommand)
    }


    /**
     * Adds several commands that will run at the same time, all stopping as soon as the first command specified finishes.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelDeadlineGroup
     */
    public fun runParallelUntilFirstCommandFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        var commandsSet = commands.toMutableSet() + builder.commands
        val firstCommand = commandsSet.first()
        commandsSet = commandsSet.minus(firstCommand)
        this.commands.removeAll(commandsSet)
        var command: Command = ParallelDeadlineGroup(firstCommand, *commandsSet.map{it.asProxy()}.toTypedArray()).finallyDo(builder.endBehavior)
        if (builder.requirements.size > 0){
            command = command.withExtraRequirements(*builder.requirements.toTypedArray())
        }
        builder.addingCommandsLocked = true
        return command.also(::addCommand)
    }

    /**
     * Adds several commands that will run at the same time, only finishing once all are complete.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelCommandGroup
     */
    public fun runParallelUntilAllFinish(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        var command: Command = ParallelCommandGroup(*commandsSet.map{it.asProxy()}.toTypedArray()).finallyDo(builder.endBehavior)
        if (builder.requirements.size > 0){
            command = command.withExtraRequirements(*builder.requirements.toTypedArray())
        }
        builder.addingCommandsLocked = true
        return command.also(::addCommand)
    }



    /**
     * Adds several commands that will run one after another.
     *
     * @param block a builder to create the commands to run sequentially
     * @see SequentialCommandGroup
     */
    public fun runSequentially(vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        var command: Command = SequentialCommandGroup(*commandsSet.map{it.asProxy()}.toTypedArray()).finallyDo(builder.endBehavior)
        if (builder.requirements.size > 0){
            command = command.withExtraRequirements(*builder.requirements.toTypedArray())
        }
        builder.addingCommandsLocked = true
        return command.also(::addCommand)
    }

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param command the command to run
     * @param timeInterval the maximum allowed runtime of the command
     */
    public fun loopFor(timeInterval: Time, command: Command): ParallelRaceGroup =
        command
        .withTimeout(timeInterval.inUnit(seconds))
        .also(::addCommand)

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param timeInterval the maximum allowed runtime of the command
     * @param requirements the Subsystems this command requires
     * @param execute the code to be run
     */
    public inline fun loopFor(timeInterval: Time, vararg requirements: Subsystem, crossinline execute: CodeBlockContext.() -> Unit): ParallelRaceGroup =
        loopFor(timeInterval, RunCommand(*requirements) { CodeBlockContext.execute() })

    /**
     * Adds a command to be run continuously.
     *
     * @param requirements the Subsystems this command requires
     * @param execute the code to be run
     */
    public fun loopForever(vararg requirements: Subsystem, execute: CodeBlockContext.() -> Unit): RunCommand =
            RunCommand(*requirements) { CodeBlockContext.execute() }.also(::addCommand)

    /**
     * Adds a command that does nothing for a specified [timeInterval], then completes.
     *
     * Useful if a delay is needed between two commands in a [SequentialCommandGroup].
     * Note that running this in parallel with other commands is unlikely to be useful.
     */
    public fun waitFor(timeInterval: Time): WaitCommand =
        WaitCommand(timeInterval.inUnit(seconds)).also(::addCommand)

    /**
     * Adds a command that does nothing until a [condition] is met, then completes.
     *
     * Useful if some condition must be met before proceeding to the next command in a [SequentialCommandGroup].
     * Note that running this in parallel with other commands is unlikely to be useful.
     */
    public fun waitUntil(condition: () -> Boolean): WaitUntilCommand =
        WaitUntilCommand(condition).also(::addCommand)

    /**
     * Adds a command that prints a message.
     *
     * @param message a function that generates the message to print
     */
    public fun printToConsole(message: () -> Any?): Command =
        InstantCommand { println(message()) }.also(::addCommand)

    /**
     * Adds a command that gets a property when it executes; this is useful for
     * getting initial positions, etc. when commands first run.
     *
     * Returns a property delegate; see [here](https://kotlinlang.org/docs/delegated-properties.html#standard-delegates)
     * for an explanation of property delegates.
     */
    public fun <T : Any> getOnceDuringRun(get: CodeBlockContext.() -> T) : ReadOnlyProperty<Any?, T> =
        DuringRunGetter(get)

    private inner class DuringRunGetter<T : Any>(private val get: CodeBlockContext.() -> T) : ReadOnlyProperty<Any?, T> {
        init {
            addCommand(
                object : Command() { // Add a new command that initializes this value in its initialize() function.
                    override fun initialize() {
                        if (!::value.isInitialized) {
                            initializeValue()
                        }
                    }

                    override fun isFinished(): Boolean = ::value.isInitialized
                }
            )
        }

        private fun initializeValue() {
            value = CodeBlockContext.get()
        }

        private lateinit var value: T

        override fun getValue(thisRef: Any?, property: KProperty<*>): T =
            try {
                value
            } catch (e: UninitializedPropertyAccessException) { // If value is tried to be used before the initializer command runs (for example, by another command running in parallel), then initialize it immediately.
                initializeValue()
                value
            }
    }



}




/**
 * Utility functions used for logging commands within the buildCommand DSL.
 */

@PublishedApi
internal fun loggedSequentialCommandGroup(name: String, vararg commands: Command): SequentialCommandGroup{
    val loggedCommands: Array<Command> = commands.map{it.withLogInCommandGroup(name)}.toTypedArray()
    return SequentialCommandGroup(
        *loggedCommands
    ).also{
        it.name = name
    }
}

internal fun Command.withLogInCommandGroup(commandGroupName: String): Command{
    fun logCommand(active: Boolean) = InstantCommand{
        Logger.recordOutput(
            "/ActiveCommands/Subcommands Of: $commandGroupName/$name",active
        )
    }

    // uses custom infix "then" operator(more concise way to do andThen)
    return logCommand(true) then this then logCommand(false)
}

/**
 * This object serves to restrict the scope of runOnce, loopForever, etc. blocks within the buildCommand.
 *
 * This discourages scenarios like this:
 * ```
 * buildCommand{
 *      loopForever{
 *          loopForever{
 *              println("hi")
 *          }
 *      }
 * }
 * ```
 * Which would factually do nothing due to the command already being created when buildCommand is initialized.
 */
@CommandBuilderMarker
public object CodeBlockContext