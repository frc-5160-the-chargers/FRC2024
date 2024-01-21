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
import kotlin.properties.ReadWriteProperty
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
 * @param block The entry point to the DSL. Has the context of [CommandBuilder].
 */
public inline fun buildCommand(
    name: String = "Generic BuildCommand",
    logIndividualCommands: Boolean = false,
    block: BuildCommandScope.() -> Unit
): Command{
    val builder = BuildCommandScope().apply(block)

    val commandArray: Array<Command> = builder.commands.toTypedArray()

    var command: Command = if(logIndividualCommands){
        loggedSequentialCommandGroup(name, *commandArray)
    }else{
        SequentialCommandGroup(*commandArray).withName(name)
    }.finallyDo(builder.endBehavior)

    if (builder.requirements.size > 0){
        command = command.withExtraRequirements(*builder.requirements.toTypedArray())
    }

    builder.addingCommandsLocked = true
    builder.addingRequirementsLocked = true

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
 * A scope exclusive to [buildCommand]; this contains things like end behavior and command requirements
 * which aren't used in other places where a [CommandBuilder] scope is asked for
 * (like runSequentially, runParallelUntilAllFinish, etc.)
 */
@CommandBuilderMarker
public class BuildCommandScope: CommandBuilder(){
    public val requirements: MutableList<Subsystem> = mutableListOf()

    public var addingRequirementsLocked: Boolean = false

    public var endBehavior: (Boolean) -> Unit = {}

    /**
     * Adds subsystems that are required across the entire [buildCommand].
     */
    public fun addRequirements(vararg requirements: Subsystem){
        if (addingRequirementsLocked){
            error("""
                It looks like you are attempting to add requirements to the command while it is running.
                This is not allowed; you must call addRequirements() within the block of the buildCommand, and not in a CodeBlockContext.
                
                For instance:
                buildCommand{
                    runOnce{
                        // does not compile
                        addRequirements(...)
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
     * Runs the function block when the [buildCommand] is finished.
     */
    public inline fun onEnd(crossinline run: CodeBlockContext.(Boolean) -> Unit){
        endBehavior = { CodeBlockContext.run(it) }
    }
}



/**
 * The scope class responsible for governing the BuildCommand DSL.
 */
@CommandBuilderMarker
public open class CommandBuilder{
    public var commands: LinkedHashSet<Command> = linkedSetOf() // LinkedHashSet keeps commands in order, but also ensures they're not added multiple times

    public var addingCommandsLocked: Boolean = false

    public fun addCommand(c: Command){
        if (addingCommandsLocked){
            error(
                """
                It seems that you have attempted to add a command to the CommandBuilder during runtime. This is not allowed.
                Make sure that you don't have any nested runOnce, loopForever, loopUntil, etc. blocks 
                within another command-adding block, and no explicit builtCommand receivers are called.
                
                buildCommand{
                    runOnce{
                        // NOT ALLOWED
                        this@buildCommand.runOnce{
                            doSomething()
                        }
                        // Will not compile(DSL scope enforcement)
                        runOnce{
                            doSomethingElse()
                        }
                    }
                } 
                """.trimIndent()
            )
        }else{
            commands.add(c)
        }
    }

    /**
     * Applies a generic modifier to a command.
     * This function must be used for command-returning functions in order for the new versions to be added to the command builder.
     * For instance:
     *
     * ```
     * buildCommand{
     *      runOnce{ println("hi") }.modify{ withTimeout(5) }
     *
     * }
     */
    public fun Command.modify(modifier: Command.() -> Command): Command{
        commands.remove(this)
        val newCommand = this.modifier()
        commands.add(newCommand)
        return newCommand
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
        commands.remove(this)
        return this
    }


    /**
     * Allows removal of commands within a code block.(For getOnceDuringRun)
     */
    context(CodeBlockContext)
    public operator fun <C: Command> C.unaryMinus(): C{
        with (this@CommandBuilder){
            commands.remove(this@unaryMinus)
        }
        return this
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
    public inline fun runParallelUntilOneFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return ParallelRaceGroup(*commandsSet.toTypedArray()).also(::addCommand)
    }


    /**
     * Adds several commands that will run at the same time, all stopping as soon as the first command specified finishes.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelDeadlineGroup
     */
    public inline fun runParallelUntilFirstCommandFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        val firstCommand = commandsSet.first()
        val otherCommands = commandsSet - firstCommand
        return ParallelDeadlineGroup(firstCommand, *otherCommands.toTypedArray()).also(::addCommand)
    }

    /**
     * Adds several commands that will run at the same time, only finishing once all are complete.
     *
     * @param commands commands to run in parallel
     * @param block a builder allowing more parallel commands to be defined and added
     * @see ParallelCommandGroup
     */
    public inline fun runParallelUntilAllFinish(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return ParallelCommandGroup(*commandsSet.toTypedArray()).also(::addCommand)
    }



    /**
     * Adds several commands that will run one after another.
     *
     * @param block a builder to create the commands to run sequentially
     * @see SequentialCommandGroup
     */
    public inline fun runSequentially(vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return SequentialCommandGroup(*commandsSet.toTypedArray()).also(::addCommand)
    }

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param command the command to run
     * @param timeInterval the maximum allowed runtime of the command
     */
    public fun runFor(timeInterval: Time, command: Command): ParallelRaceGroup =
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
    public inline fun loopFor(
        timeInterval: Time,
        vararg requirements: Subsystem,
        crossinline execute: CodeBlockContext.() -> Unit
    ): ParallelRaceGroup =
        RunCommand(*requirements) { CodeBlockContext.execute() }.withTimeout(timeInterval.inUnit(seconds)).also(::addCommand)

    /**
     * Adds a command to be run continuously.
     *
     * @param requirements the Subsystems this command requires
     * @param execute the code to be run
     */
    public inline fun loop(
        vararg requirements: Subsystem,
        crossinline execute: CodeBlockContext.() -> Unit
    ): RunCommand =
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
     * Creates values that will refresh once during run;
     * at the point of which this statement is placed within the command.
     *
     * In order to do this, read-only properties(val)
     * within the command builder block have their getValue function "delegated" by this function.
     *
     * See [here](https://kotlinlang.org/docs/delegated-properties.html#standard-delegates)
     * for an explanation of property delegates.
     *
     * ```
     * val command = buildCommand{
     *      val armStartingPosition by getOnceDuringRun{ arm.position }
     *
     *      runOnce{
     *          // because this is called in a function(CodeBlockContext) block,
     *          // it will print the new armStartingPosition whenever the command runs.
     *          println(armStartingPosition)
     *      }
     *
     *      // this fetches the value of armStartingPosition when the command is initialized;
     *      // this means that p2 will not refresh itself when the command runs, unlike armStartingPosition.
     *      val p2 = armStartingPosition
     *
     *      // this, on the other hand, is valid.
     *      val p2 by getOnceDuringRun{ armStartingPosition }
     * }
     */
    public fun <T : Any> getOnceDuringRun(get: CodeBlockContext.() -> T) : ReadOnlyProperty<Any?, T> =
        object: ReadOnlyProperty<Any?, T> {
            init {
                addCommand(
                    object : Command() { // Add a new command that initializes this value in its initialize() function.
                        override fun initialize() {
                            // if the value is not initialized yet, initialize it.
                            // ::value is a property reference, which allows you to check whether something
                            if (!::value.isInitialized) {
                                initializeValue()
                            }
                        }

                        // command does not stop until value is initialized
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


    /**
     *
     * Adds a command that resets a variable when it is run.
     *
     * Returns a property delegate; see [here](https://kotlinlang.org/docs/delegated-properties.html#standard-delegates)
     * for an explanation of property delegates.
     */
    public fun <T: Any> resetDuringRun(getDefault: () -> T): ReadWriteProperty<Any?, T> =
        object: ReadWriteProperty<Any?, T>{
            private var value: T = getDefault()

            init{
                addCommand(
                    // adds a command that resets the value
                    InstantCommand({value = getDefault()})
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): T = value
            override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) { this.value = value }
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
 *          // will not compile due to the marker prohibiting implicit "this"
 *          // when not called directly within buildCommand.
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