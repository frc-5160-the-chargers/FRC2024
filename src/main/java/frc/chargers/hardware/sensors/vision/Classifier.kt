@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

/**
 * Represents a generic system that can report a classified result of any kind.
 */
public interface Classifier<T> {
    /**
     * The result obtained from the classifier.
     */
    public val itemType: T

    /**
     * Resets the pipeline of the overarching vision camera to the appropriate value
     * for this pipeline.
     */
    public fun reset()

    /**
     * Requires the overarching vision camera
     * of the pipeline.
     */
    public fun require()

    /**
     * Removes the requirement of the overarching vision camera
     * of the pipeline.
     */
    public fun removeRequirement()
}