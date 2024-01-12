@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.framework

/**
 * A generic data class representing Git data, such as the SHA, branch, etc.
 */
public data class GitData(
    val projectName: String,
    val buildDate: String,
    val sha: String,
    val branch: String,
    val dirty: Int
)

