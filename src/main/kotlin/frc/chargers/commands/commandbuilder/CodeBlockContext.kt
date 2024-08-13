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
class CodeBlockContext {
    private var currentBlockStopped = false
    private var overallCommandStopped = false

    /**
     * Declares that the current block(runOnce, loopWhile, loopUntil)
     * has finished it's exeuction. Equivalent to setting the isFinished of the current
     * command block to true.
     *
     * The respective block will first continue executing the rest of the code,
     * then move on to the next block within the buildCommand.
     *
     * To obtain break statement-like behavior, it is recommended to pair this statement
     * with a qualified return statement, like so:
     *
     * ```
     * buildCommand{
     *      loop{
     *          if (condMet){
     *              declareBlockFinished()
     *              return@loop
     *          }
     *      }
     *
     *      loopFor(5.seconds){
     *          if (condMet){
     *              declareBlockFinished()
     *              return@loopFor // loopFor will entirely stop running
     *          }
     *      }
     * }
     * ```
     */
    fun declareBlockFinished() {
        currentBlockStopped = true
    }

    /**
     * Similar to [declareBlockFinished];
     * however, this function sets the "isFinished"
     * of the entire buildCommand to true.
     *
     * @see declareBlockFinished
     */
    fun declareCommandFinished() {
        overallCommandStopped = true
    }

    @PublishedApi
    internal fun currentBlockStopped(): Boolean {
        if (currentBlockStopped){
            currentBlockStopped = false
            return true
        }else{
            return false
        }
    }

    @PublishedApi
    internal fun commandStopped(): Boolean = overallCommandStopped
}