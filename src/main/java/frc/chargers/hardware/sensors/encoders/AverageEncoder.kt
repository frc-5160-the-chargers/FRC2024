@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import frc.chargers.hardware.motorcontrol.EncoderMotorController

/**
 * An abstraction of multiple encoders into one.
 *
 * The output of its position and velocity are the average
 * of the positions and velocities of its composite encoders.
 */
public class AverageEncoder(private vararg val encoders: Encoder) : Encoder {
    public constructor(vararg motorControllers: EncoderMotorController) :
            this(*motorControllers.map(EncoderMotorController::encoder).toTypedArray())

    override val angularPosition: Angle
        get() = encoders.map(Encoder::angularPosition).average()

    override val angularVelocity: AngularVelocity
        get() = encoders.map(Encoder::angularVelocity).average()
}