@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.commands.commandbuilder

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.*
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

/**
 * The scope class responsible for governing the BuildCommand DSL.
 */
@CommandBuilderMarker
public open class CommandBuilder {
    @PublishedApi
    internal var commands: LinkedHashSet<Command> = linkedSetOf() // LinkedHashSet keeps commands in order, but also ensures they're not added multiple times

    @PublishedApi
    internal var addingCommandsLocked: Boolean = false

    @PublishedApi
    internal val context = CodeBlockContext()

    @PublishedApi
    internal fun addCommand(c: Command) {
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
     *      loop{ println("hi") }.modify{ it,withTimeout(5) }
     *
     * }
     */
    public fun Command.modify(modifier: (Command) -> Command): Command {
        commands.remove(this)
        val newCommand = modifier(this)
        commands.add(newCommand)
        return newCommand
    }


    
    /**
     * Adds a single command to be run until its completion.
     * Usage Example:
     * ```
     * class ArmCommand(val angle: Angle, val arm: Arm): Command(){...}
     *
     * val command = buildCommand{
     *      +ArmCommand(2.0.radians, armSubsystem)
     * }
     * ```
     */
    public operator fun <C : Command> C.unaryPlus(): C {
        addCommand(this)
        return this
    }

    /**
     * Returns a command, then removes it from the set of commands within the [CommandBuilder].
     *
     * ```
     * buildCommand{
     *    val command by getOnceDuringRun{
     *      -runOnce{ doSomethingHere()} // will no longer be added to the command builder
     *    }
     * }
     * ```
     */
    public operator fun <C: Command> C.unaryMinus(): C {
        commands.remove(this)
        return this
    }

    /**
     * Adds a command that will run once and then complete.
     *
     * Equivalent to an [InstantCommand].
     *
     * @param execute the code to be run
     */
    public fun runOnce(execute: CodeBlockContext.() -> Unit): Command =
        InstantCommand({ context.execute() }).also(::addCommand)


    /**
     * Adds a command that will run the command onTrue or only if a condition is met.
     *
     * Equivalent to a [ConditionalCommand].
     *
     * @param condition the condition supplier utilized.
     * @param onTrue the command ran when the condition returns true.
     * @param onFalse the command ran when the condition returns false; defaults to an empty command.
     */
    public fun runIf(condition: () -> Boolean, onTrue: Command, onFalse: Command = InstantCommand()): Command =
        ConditionalCommand(onTrue, onFalse, condition).also{
            addCommand(it)
            this.commands.remove(onTrue)
            this.commands.remove(onFalse)
        }

    /**
     * Adds a command that will run the appropriate mapped command, depending on the key given.
     *
     * Equivalent to a [SelectCommand].
     *
     * @param key: A lambda that gets a generic value, used for choosing an appropriate command.
     * @param commands: A map between the key and commands to be called.
     */
    public fun <T: Any> runWhen(key: () -> T, commands: Map<T, Command>): Command =
        SelectCommand(commands, key).also{
            addCommand(it)
            this.commands.removeAll(commands.values.toSet())
        }


    /**
     * Adds a command that will run *until* the [condition] is met.
     *
     * Equivalent to the [Command.until] decorator.
     *
     * @param condition the condition to be met
     * @param command the command to run until [condition] is met
     */
    public fun runUntil(condition: () -> Boolean, command: Command): Command =
        command.until(condition)
            .also{
                addCommand(it)
                this.commands.remove(command)
            }

    /**
     * Adds a command that will run *while* the [condition] is met.
     *
     * Equivalent to the [Command.onlyWhile] decorator.
     *
     * @param condition the condition to be met
     * @param command the command to run until [condition] is met
     */
    public fun runWhile(condition: () -> Boolean, command: Command): Command =
        command.onlyWhile(condition)
            .also{
                addCommand(it)
                this.commands.remove(command)
            }

    /**
     * Adds a command that will run the code block repeatedly *until* the [condition] is met.
     *
     * @param condition the condition to be met
     * @param execute the code to be run until [condition] is met
     */
    public inline fun loopUntil(crossinline condition: () -> Boolean, crossinline execute: CodeBlockContext.() -> Unit): Command =
        RunCommand({ context.execute() })
            .until{ condition() || context.currentBlockStopped() }
            .also(::addCommand)

    /**
     * Adds a command that will run *while* [condition] is true.
     *
     * @param condition the condition to be met
     * @param execute the code to be run
     */
    public inline fun loopWhile(crossinline condition: () -> Boolean, crossinline execute: CodeBlockContext.() -> Unit): Command =
        RunCommand({ context.execute() })
            .onlyWhile{ condition() && !context.currentBlockStopped() }
            .also(::addCommand)

    /**
     * Adds several commands that will run at the same time, all stopping as soon as one finishes.
     * These commands can either be specified as a function parameter, or as a builder block
     * within the command builder context [block].
     *
     * Equivalent to a [ParallelRaceGroup]
     *
     * @param commands commands to run in parallel;
     * these are automatically removed from the overarching builder(so you can use runOnce/loopUntil).
     * @param block a builder allowing more parallel commands to be defined and added
     */
    public inline fun runParallelUntilOneFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return ParallelRaceGroup(*commandsSet.toTypedArray()).also(::addCommand)
    }


    /**
     * Adds several commands that will run at the same time, all stopping as soon as the first command specified finishes.
     * These commands can either be specified as a function parameter, or as a builder block
     * within the command builder context [block].
     *
     * Equivalent to a [ParallelDeadlineGroup].
     *
     * @param commands commands to run in parallel;
     * these are automatically removed from the overarching builder(so you can use runOnce/loopUntil).
     * @param block a builder allowing more parallel commands to be defined and added
     */
    public inline fun runParallelUntilFirstCommandFinishes(vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        try{
            val firstCommand = commandsSet.first()
            val otherCommands = commandsSet - firstCommand
            return ParallelDeadlineGroup(firstCommand, *otherCommands.toTypedArray()).also(::addCommand)
        }catch(e: NoSuchElementException){
            throw NoSuchElementException("Your runParallelUntilFirstCommandFinishes needs to have a first command(or deadline); however, you have not specified any commands.")
        }
    }

    /**
     * Adds several commands that will run at the same time, only finishing once all are complete.
     * These commands can either be specified as a function parameter, or as a builder block
     * within the command builder context [block].
     *
     * Equivalent to a [ParallelCommandGroup].
     *
     * @param commands commands to run in parallel;
     * these are automatically removed from the overarching builder(so you can use runOnce/loopUntil).
     * @param block a builder allowing more parallel commands to be defined and added
     */
    public inline fun runParallelUntilAllFinish(vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return ParallelCommandGroup(*commandsSet.toTypedArray()).also(::addCommand)
    }



    /**
     * Adds several commands that will run one after another.
     * These commands can either be specified as a function parameter, or as a builder block
     * within the command builder context [block].
     *
     * Equivalent to a [SequentialCommandGroup].
     *
     * @param commands explicitly specified commands to be run sequentially;
     * these are automatically removed from the overarching builder(so you can use runOnce/loopUntil).
     * @param block a builder to create the commands to run sequentially
     */
    public inline fun runSequentially(vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return SequentialCommandGroup(*commandsSet.toTypedArray()).also(::addCommand)
    }

    /**
     * Runs certain commands sequentially for a certain [Time].
     */
    public inline fun runSequentiallyFor(time: Time, vararg commands: Command, block: CommandBuilder.() -> Unit = {}): Command {
        val sequentialCommand = runSequentially(*commands, block = block)
        this.commands.remove(sequentialCommand)
        return sequentialCommand.withTimeout(time.inUnit(seconds)).also(::addCommand)
    }

    /**
     * Runs certain commands sequentially **until** a certain condition is met.
     */
    public inline fun runSequentiallyUntil(
        noinline condition: () -> Boolean,
        vararg commands: Command,
        block: CommandBuilder.() -> Unit = {}
    ): Command {
        val sequentialCommand = runSequentially(*commands, block = block)
        this.commands.remove(sequentialCommand)
        return sequentialCommand.unless(condition).also(::addCommand)
    }

    /**
     * Runs certain commands sequentially **until** a certain condition is met.
     */
    public inline fun runSequentiallyWhile(
        crossinline condition: () -> Boolean,
        vararg commands: Command,
        block: CommandBuilder.() -> Unit = {}
    ): Command =
        runSequentiallyUntil({ !condition() }, *commands, block = block)


    /**
     * Adds a command that will schedule the given commands provided
     * instead of running them sequentially within the command group itself.
     *
     * This is useful for forking off of command groups.
     *
     *
     * @param block a builder to create the commands to run sequentially
     * @see ScheduleCommand
     */
    public inline fun runSeparately(vararg commands: Command, block: CommandBuilder.() -> Unit): Command {
        val builder = CommandBuilder().apply(block)
        val commandsSet = commands.toMutableSet() + builder.commands
        this.commands.removeAll(commandsSet)
        builder.addingCommandsLocked = true
        return ScheduleCommand(*commandsSet.toTypedArray()).also(::addCommand)
    }


    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * Equivalent to the [Command.withTimeout] decorator.
     *
     * @param command the command to run
     * @param timeInterval the maximum allowed runtime of the command
     */
    public fun runFor(timeInterval: Time, command: Command): Command =
        command
        .withTimeout(timeInterval.inUnit(seconds))
        .also(::addCommand)

    /**
     * Adds a command that will run until either the [timeInterval] expires or it completes on its own.
     *
     * @param timeInterval the maximum allowed runtime of the command
     * @param execute the code to be run
     */
    public inline fun loopFor(timeInterval: Time, crossinline execute: CodeBlockContext.() -> Unit): Command =
        RunCommand({ context.execute() })
            .withTimeout(timeInterval.inUnit(seconds))
            .until(context::currentBlockStopped)
            .also(::addCommand)

    /**
     * Adds a command to be run continuously.
     *
     * Equivalent to a [RunCommand].
     *
     * @param execute the code to be run
     */
    public inline fun loop(crossinline execute: CodeBlockContext.() -> Unit): Command =
        RunCommand({ context.execute() })
            .until(context::currentBlockStopped)
            .also(::addCommand)

    /**
     * Adds a command that does nothing for a specified [timeInterval], then completes.
     *
     * Useful if a delay is needed between two commands in a [SequentialCommandGroup].
     */
    public fun waitFor(timeInterval: Time): WaitCommand =
        WaitCommand(timeInterval.inUnit(seconds)).also(::addCommand)

    /**
     * Adds a command that does nothing until a [condition] is met, then completes.
     *
     * Useful if some condition must be met before proceeding to the next command in a [SequentialCommandGroup].
     */
    public fun waitUntil(condition: () -> Boolean): WaitUntilCommand =
        WaitUntilCommand(condition).also(::addCommand)

    /**
     * Creates a value that will refresh once during run;
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
     *      // this fetches the value of armStartingPosition when the buildCommand is created
     *      // this means that p2 will not refresh itself when the command runs, unlike armStartingPosition.
     *      val p2 = arm.position
     *
     *      // this, on the other hand, is valid.
     *      val p2 by getOnceDuringRun{ arm.position }
     *
     *      runOnce{
     *          // because this is called in a function block(CodeBlockContext),
     *          // it will print the new armStartingPosition whenever the command runs.
     *          println(armStartingPosition)
     *      }
     * }
     */
    public fun <T : Any> getOnceDuringRun(get: CodeBlockContext.() -> T) : ReadOnlyProperty<Any?, T> =
        object: ReadOnlyProperty<Any?, T> {
            private lateinit var value: T

            private var hasInitialized: Boolean = false

            private fun initializeValue() {
                value = context.get()
                hasInitialized = true
            }

            init {
                addCommand(
                    object : Command() { // Add a new command that initializes this value in its initialize() function.
                        override fun initialize() {
                            // if the value is not initialized yet, initialize it.
                            if (!hasInitialized) {
                                initializeValue()
                            }
                        }

                        // command does not stop until value is initialized
                        override fun isFinished(): Boolean = hasInitialized

                        override fun end(interrupted: Boolean) {
                            // sets hasInitialized to false when command ends,
                            // so that the value can re-initialize when the command is executed again.
                            hasInitialized = false
                        }
                    }
                )
            }

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
     * Adds a command that resets a variable to it's default(provided by getDefault) when it is run.
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
                    InstantCommand({ value = getDefault() })
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): T = value

            override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) { this.value = value }
        }
}

