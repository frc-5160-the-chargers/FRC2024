@file:Suppress("unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.reduxrobotics.sensors.canandmag.Canandmag

class ChargerCanandmag(val deviceID: Int, settings: Canandmag.Settings = Canandmag.Settings()): Encoder {
    val base = Canandmag(deviceID)

    init {
        base.settings = settings
    }

    val absolute: Encoder = AbsoluteEncoderAdaptor()
    private inner class AbsoluteEncoderAdaptor: Encoder by this {
        override val angularPosition: Angle
            get() = base.absPosition.ofUnit(radians)
    }

    override val angularVelocity: AngularVelocity
        get() = base.velocity.ofUnit(rotations / seconds)

    override val angularPosition: Angle
        get() = base.position.ofUnit(rotations)
}

fun Canandmag.Settings.setZeroOffset(zeroOffset: Angle): Canandmag.Settings =
    setZeroOffset(zeroOffset.inUnit(rotations))