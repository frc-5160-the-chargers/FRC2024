package frc.chargers.commands.commandbuilder

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
object CodeBlockContext