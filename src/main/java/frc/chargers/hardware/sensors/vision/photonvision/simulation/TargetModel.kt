@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision.photonvision.simulation

import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import org.photonvision.estimation.TargetModel

fun TargetModel(
    width: Length,
    height: Length
): TargetModel =
    TargetModel(width.inUnit(meters), height.inUnit(meters))