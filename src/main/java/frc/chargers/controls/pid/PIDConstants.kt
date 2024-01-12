@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.pid


/**
 * A data class representing the various constants needed to configure a PID controller.
 */
public data class PIDConstants(
    /**
     * The constant that weights the proportional PID term.
     */
    @JvmField
    public var kP: Double,

    /**
     * The constant that weights the integral PID term.
     */
    @JvmField
    public var kI: Double,

    /**
     * The constant that weights the derivative PID term.
     */
    @JvmField
    public var kD: Double
){
    public companion object{
        public val None: PIDConstants = PIDConstants(0.0,0.0,0.0)
    }
}
