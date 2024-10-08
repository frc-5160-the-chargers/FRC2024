@file:Suppress("unused")
package frc.chargers.hardware.sensors.encoders

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.reduxrobotics.sensors.canandmag.Canandmag


/**
 * A wrapper around the [Canandmag] class that implements the [Encoder] interface.
 */
class ChargerCanandmag(
    val deviceID: Int,
    settings: Canandmag.Settings = Canandmag.Settings(),
    private val useAbsolutePosition: Boolean = true,
): Encoder {
    val base = Canandmag(deviceID)

    init {
        base.settings = settings
    }

    override val angularVelocity: AngularVelocity
        get() = base.velocity.ofUnit(rotations / seconds)

    override val angularPosition: Angle
        get() = (if (useAbsolutePosition) base.position else base.absPosition).ofUnit(rotations)
}

fun Canandmag.Settings.setZeroOffset(zeroOffset: Angle): Canandmag.Settings =
    setZeroOffset(zeroOffset.inUnit(rotations))