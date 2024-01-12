@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.controls.feedforward

import com.batterystaple.kmeasure.quantities.*
import frc.chargers.utils.math.units.*


/**
 * Represents constants for Feedforward control
 * of an angular velocity targeting mechanism with no gravity.
 */
public data class AngularMotorFFConstants(
    val kS: Voltage,
    val kV: VoltagePerAngularVelocity,
    val kA: VoltagePerAngularAcceleration = VoltagePerAngularAcceleration(0.0)
){
    public companion object{
        public val None: AngularMotorFFConstants = AngularMotorFFConstants(Quantity(0.0), Quantity(0.0))

        public fun fromSI(
            kS: Double,
            kV: Double,
            kA: Double
        ): AngularMotorFFConstants =
            AngularMotorFFConstants(
                Voltage(kS),
                VoltagePerAngularVelocity(kV),
                VoltagePerAngularAcceleration(kA)
            )
    }
}




/**
 * Represents constants for Feedforward control
 * of an angular velocity targeting mechanism with no gravity.
 */
public data class LinearMotorFFConstants(
    val kS: Voltage,
    val kV: VoltagePerVelocity,
    val kA: VoltagePerAcceleration = VoltagePerAcceleration(0.0)
){
    public companion object{
        public val None: LinearMotorFFConstants = LinearMotorFFConstants(Quantity(0.0), Quantity(0.0))

        public fun fromSI(
            kS: Double,
            kV: Double,
            kA: Double
        ): LinearMotorFFConstants =
            LinearMotorFFConstants(
                Voltage(kS),
                VoltagePerVelocity(kV),
                VoltagePerAcceleration(kA)
            )
    }

}

/**
 * Holds the nessecary Feed forward constants
 * to set a certain angular velocity for an arm mechanism.
 */
public data class ArmFFConstants(
    val kS: Voltage,
    val kG: VoltagePerAngle,
    val kV: VoltagePerAngularVelocity,
    val kA: VoltagePerAngularAcceleration = VoltagePerAngularAcceleration(0.0)
){
    public companion object{
        public val None: ArmFFConstants = ArmFFConstants(Quantity(0.0), Quantity(0.0), Quantity(0.0))

        public fun fromSI(
            kS: Double,
            kG: Double,
            kV: Double,
            kA: Double
        ): ArmFFConstants =
            ArmFFConstants(
                Voltage(kS),
                VoltagePerAngle(kG),
                VoltagePerAngularVelocity(kV),
                VoltagePerAngularAcceleration(kA)
            )
    }
}


/**
 * Represents constants for Feedforward control
 * of a linear velocity targeting mechanism.
 */
public data class ElevatorFFConstants(
    val kS: Voltage,
    val kG: Voltage,
    val kV: VoltagePerVelocity,
    val kA: VoltagePerAcceleration = VoltagePerAcceleration(0.0)
){
    public companion object{
        public val None: ElevatorFFConstants = ElevatorFFConstants(
            Quantity(0.0), Quantity(0.0), Quantity(0.0)
        )

        public fun fromSI(
            kS: Double,
            kG: Double,
            kV: Double,
            kA: Double
        ): ElevatorFFConstants =
            ElevatorFFConstants(
                Voltage(kS),
                Voltage(kG),
                VoltagePerVelocity(kV),
                VoltagePerAcceleration(kA)
            )
    }

}

