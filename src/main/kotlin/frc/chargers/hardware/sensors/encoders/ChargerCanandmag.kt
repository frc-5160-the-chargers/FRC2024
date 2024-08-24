@file:Suppress("unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.reduxrobotics.sensors.canandmag.Canandmag

class ChargerCanandmag(id: Int): Canandmag(id), Encoder{
    val absolute: Encoder = AbsoluteEncoderAdaptor()
    private inner class AbsoluteEncoderAdaptor: Encoder by this {
        override val angularPosition: Angle
            get() = getAbsPosition().ofUnit(radians)
    }

    override val angularVelocity: AngularVelocity
        get() = getVelocity().ofUnit(rotations / seconds)

    override val angularPosition: Angle
        get() = getPosition().ofUnit(rotations)
}