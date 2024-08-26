package frc.chargers.commands.commandbuilder

/**
 * A command request, to be returned within a [BuildCommandScope.loop]
 * or [BuildCommandScope.loopFor] block.
 */
enum class Request {
    CONTINUE,
    BREAK,
    STOP_COMMAND,
}