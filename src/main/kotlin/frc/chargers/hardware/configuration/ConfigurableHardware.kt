@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.configuration

/**
 * An interface used to denote a piece of hardware(motor controllers, encoders, etc.)
 * that can be configured.
 */
public interface ConfigurableHardware<in C: HardwareConfiguration> {
    public fun configure(configuration: C)
}